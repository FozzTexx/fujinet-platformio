#!/usr/bin/env python3
"""
check_gitignore_policy.py - fail if any .gitignore hides things inside source dirs.

Rule enforced: build output must never land in the source tree.  A "source
directory" is any directory that contains at least one git-TRACKED file.  If a
.gitignore anywhere in the repo ignores (or could ignore) something inside such
a directory, that is a violation.  Ignored output belongs in dedicated build
directories that contain no tracked files.

Static checks (always run, work on a clean checkout):
  1. Directory check - asks git itself (`git check-ignore --no-index`) whether
     any source directory is matched by an ignore rule, e.g. `bin/` swallowing
     tools/bin/.
  2. Pattern check   - every pattern in every .gitignore is resolved to the
     place it applies.  Anchored literal paths (`/src/gen/`, `src/foo.c`) are
     resolved exactly; globs use their literal prefix (`src/**/*.o` -> src/);
     unanchored patterns apply to the directory holding the .gitignore.  Any
     that land in or under a source directory are violations.  So a nested
     .gitignore inside a source directory is itself a violation.

Dynamic check (--check-files, run it AFTER a build):
  3. Lists untracked and ignored files/dirs that actually exist inside source
     directories, and tracked files the build modified or deleted - i.e. proof
     that the build changed the source tree.

Exemptions (for vendor code etc.):
  Put rules, one per line, in <repo-root>/.gitignore-check-exempt
  (or pass --exempt-file / --exempt PATTERN; both can be repeated).
      # comment
      third_party/vendorlib        # this dir and everything beneath it
      /docs/_build                 # leading slash: anchored to repo root
      vendor                       # no slash: matches a dir/file of that name at any depth
      src/legacy/*                 # fnmatch globs ('*' also crosses '/')
  An exempt path covers everything beneath it, and any .gitignore file located
  in an exempt directory is skipped entirely.

Exit codes: 0 = clean, 1 = violations found, 2 = usage / git error.
Needs only Python 3.8+ and git on PATH.
"""

import argparse
import fnmatch
import os
import posixpath
import subprocess
import sys

DEFAULT_EXEMPT_FILE = ".gitignore-check-exempt"
GLOB_CHARS = set("*?[\\")


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------
def die(msg, code=2):
    print(f"check_gitignore_policy: error: {msg}", file=sys.stderr)
    sys.exit(code)


def run_git(root, *args, stdin=None, ok=(0,)):
    try:
        p = subprocess.run(["git", "-C", root, *args], input=stdin,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except FileNotFoundError:
        die("git not found on PATH")
    if p.returncode not in ok:
        die(f"git {' '.join(args)} failed: {p.stderr.decode(errors='replace').strip()}")
    return p.stdout


def split_nul(data):
    return [os.fsdecode(x) for x in data.split(b"\0") if x]


def norm(path):
    n = posixpath.normpath(path) if path else ""
    return "" if n == "." else n


def ancestors(path):
    """path, then each parent, down to (but excluding) the root ''."""
    while path:
        yield path
        path = posixpath.dirname(path)


class Exemptions:
    def __init__(self, rules):
        self.rules = []
        for r in rules:
            r = r.split("#", 1)[0].strip()
            if not r:
                continue
            anchored = r.startswith("/") or "/" in r.rstrip("/")
            self.rules.append((r.strip("/"), anchored))

    def __call__(self, path):
        if not path:
            return False
        parts = path.split("/")
        prefixes = ["/".join(parts[:i + 1]) for i in range(len(parts))]
        for rule, anchored in self.rules:
            candidates = prefixes if anchored else parts
            if any(fnmatch.fnmatchcase(c, rule) for c in candidates):
                return True
        return False


# --------------------------------------------------------------------------
# repo inspection
# --------------------------------------------------------------------------
def tracked_source_dirs(root, strict_root):
    files = split_nul(run_git(root, "ls-files", "-z"))
    dirs = set()
    for f in files:
        d = posixpath.dirname(f)
        dirs.update(ancestors(d))
    if strict_root:
        dirs.add("")
    return dirs


def find_gitignores(root):
    files = split_nul(run_git(root, "ls-files", "-z", "--cached", "--others",
                              "--exclude-standard"))
    found = {f for f in files
             if posixpath.basename(f) == ".gitignore"
             and os.path.isfile(os.path.join(root, f))}
    return sorted(found)


class Sources:
    def __init__(self, dirs):
        self.dirs = dirs

    def owner(self, path):
        """Nearest source dir containing `path` (or equal to it), else None."""
        for a in ancestors(path):
            if a in self.dirs:
                return a
        return "" if "" in self.dirs else None


# --------------------------------------------------------------------------
# checks
# --------------------------------------------------------------------------
def check_dirs_ignored(root, sources, exempt):
    """Ask git whether any source dir is itself matched by a .gitignore rule."""
    cands = sorted(d for d in sources.dirs if d and not exempt(d))
    if not cands:
        return []
    out = run_git(root, "check-ignore", "--no-index", "-v", "-z", "--stdin",
                  stdin=("\0".join(cands) + "\0").encode(), ok=(0, 1))
    fields = split_nul(out)
    hits = {}
    for i in range(0, len(fields) - 3, 4):
        src, lineno, pattern, path = fields[i:i + 4]
        if posixpath.basename(src) != ".gitignore" or pattern.startswith("!"):
            continue  # ignore global excludes / .git/info/exclude / negations
        if exempt(src):
            continue
        hits[norm(path)] = (src, int(lineno), pattern)

    problems = []
    for path in sorted(hits):
        # report only the top-most ignored dir, not every child under it
        if any(a in hits for a in ancestors(posixpath.dirname(path))):
            continue
        src, lineno, pattern = hits[path]
        problems.append((src, lineno,
                         f"pattern '{pattern}' ignores source directory '{path}/' "
                         f"(it contains tracked files)"))
    return problems


def resolve_base(gdir, pattern):
    """Where in the tree could this gitignore pattern apply?  -> repo-relative path."""
    p = pattern.rstrip("/")
    anchored = "/" in p
    segs = [s for s in p.lstrip("/").split("/") if s]
    if not anchored:
        return gdir
    prefix = []
    for s in segs:
        if any(c in GLOB_CHARS for c in s):
            break
        prefix.append(s)
    return norm(posixpath.join(gdir, *prefix)) if prefix else gdir


def check_patterns(root, gitignores, sources, exempt):
    problems = []
    for gi in gitignores:
        if exempt(gi):
            continue
        gdir = posixpath.dirname(gi)
        with open(os.path.join(root, gi), encoding="utf-8", errors="replace") as fh:
            lines = fh.read().splitlines()
        for lineno, raw in enumerate(lines, 1):
            pat = raw.rstrip(" \t")
            if not pat.strip() or pat.startswith("#") or pat.startswith("!"):
                continue
            if pat.startswith("\\#") or pat.startswith("\\!"):
                pat = pat[1:]
            base = resolve_base(gdir, pat)
            if exempt(base):
                continue
            owner = sources.owner(base)
            if owner is None:
                continue
            where = f"'{base}'" if base else "the repo root"
            problems.append((gi, lineno,
                             f"pattern '{raw.strip()}' applies inside source directory "
                             f"'{owner}/' (resolves to {where})"))
    return problems


def check_files_present(root, sources, exempt):
    """After a build: untracked/ignored stuff that actually exists in source dirs."""
    problems = []
    for label, extra in (("ignored", ("--ignored",)), ("untracked", ())):
        out = run_git(root, "ls-files", "-z", "--others", "--exclude-standard",
                      "--directory", *extra)
        for entry in split_nul(out):
            path = norm(entry.rstrip("/"))
            parent = posixpath.dirname(path)
            if exempt(path):
                continue
            owner = sources.owner(parent)
            if owner is None:
                continue
            problems.append((path + ("/" if entry.endswith("/") else ""), 0,
                             f"{label} file/dir exists inside source directory "
                             f"'{owner}/' - the build modified the source tree"))

    # Tracked files rewritten (or removed) by the build, e.g. a regenerated header.
    deleted = set(split_nul(run_git(root, "ls-files", "-z", "--deleted")))
    modified = set(split_nul(run_git(root, "ls-files", "-z", "--modified"))) - deleted
    for label, paths in (("modified", modified), ("deleted", deleted)):
        for path in sorted(paths):
            if not exempt(path):
                problems.append((path, 0, f"tracked file {label} - the build "
                                          f"changed the source tree"))
    return problems


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="Fail if .gitignore rules cover source directories.",
        epilog="See the docstring at the top of this file for exemption syntax.")
    ap.add_argument("--repo", default=".", help="path inside the repo (default: .)")
    ap.add_argument("--exempt-file", action="append", default=[],
                    help=f"file of exemption rules (default: <root>/{DEFAULT_EXEMPT_FILE})")
    ap.add_argument("--exempt", action="append", default=[], metavar="PATTERN",
                    help="exempt a path/pattern (repeatable)")
    ap.add_argument("--check-files", action="store_true",
                    help="also verify no untracked/ignored files exist in source dirs "
                         "(run after a build)")
    ap.add_argument("--strict-root", action="store_true",
                    help="treat the repo root itself as a source dir (default: it is not, "
                         "so a root .gitignore with /build/ or *.pyc is allowed)")
    ap.add_argument("--list-source-dirs", action="store_true",
                    help="print the detected source directories and exit")
    args = ap.parse_args()

    # Derive the root from --repo plus git's relative "cdup" rather than using
    # --show-toplevel: under MSYS2 git reports /c/... paths that a native
    # Windows Python cannot open.
    base = os.path.abspath(args.repo)
    cdup = os.fsdecode(run_git(base, "rev-parse", "--show-cdup").strip())
    root = os.path.normpath(os.path.join(base, cdup)) if cdup else base

    rules = list(args.exempt)
    ex_files = args.exempt_file or [os.path.join(root, DEFAULT_EXEMPT_FILE)]
    for f in ex_files:
        if os.path.isfile(f):
            with open(f, encoding="utf-8") as fh:
                rules += fh.read().splitlines()
        elif args.exempt_file:
            die(f"exempt file not found: {f}")
    exempt = Exemptions(rules)

    sources = Sources(tracked_source_dirs(root, args.strict_root))
    if args.list_source_dirs:
        print("\n".join(sorted(sources.dirs)))
        return 0

    gitignores = find_gitignores(root)
    problems = []
    problems += check_dirs_ignored(root, sources, exempt)
    problems += check_patterns(root, gitignores, sources, exempt)
    if args.check_files:
        problems += check_files_present(root, sources, exempt)

    seen, unique = set(), []
    for p in sorted(problems):
        if p not in seen:
            seen.add(p)
            unique.append(p)

    if not unique:
        print(f"OK: checked {len(gitignores)} .gitignore file(s); "
              f"no ignore rules or build output inside source directories.")
        return 0

    for path, lineno, msg in unique:
        loc = f"{path}:{lineno}" if lineno else path
        print(f"{loc}: {msg}")
    print(f"\nFAIL: {len(unique)} violation(s). Build output must not go in source "
          f"directories. Change the build to write elsewhere, or - for vendor code "
          f"only - add an exemption to {DEFAULT_EXEMPT_FILE}.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
