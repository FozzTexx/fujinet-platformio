/* From RFC2104 */

/*
** Function: hmac_md5
*/
#include <esp_rom_md5.h>

#if !defined(_MSC_VER) && !defined(PS2_EE_PLATFORM) && !defined(PS2_IOP_PLATFORM)
#include <strings.h>
#endif

#include "compat.h"

#include "md5.h"

/*
 * unsigned char*  text;                pointer to data stream/
 * int             text_len;            length of data stream
 * unsigned char*  key;                 pointer to authentication key
 * int             key_len;             length of authentication key
 * caddr_t         digest;              caller digest to be filled in
 */
void
smb2_hmac_md5(unsigned char *text, int text_len, unsigned char *key, int key_len,
	 unsigned char *digest)
{
        md5_context_t context;
        unsigned char k_ipad[65];    /* inner padding -
                                      * key XORd with ipad
                                      */
        unsigned char k_opad[65];    /* outer padding -
                                      * key XORd with opad
                                      */
        unsigned char tk[16];
        int i;
        /* if key is longer than 64 bytes reset it to key=MD5(key) */
        if (key_len > 64) {
		md5_context_t tctx;

                esp_rom_md5_init(&tctx);
                esp_rom_md5_update(&tctx, key, key_len);
                esp_rom_md5_final(tk, &tctx);

                key = tk;
                key_len = 16;
        }

        /*
         * the HMAC_MD5 transform looks like:
         *
         * MD5(K XOR opad, MD5(K XOR ipad, text))
         *
         * where K is an n byte key
         * ipad is the byte 0x36 repeated 64 times
         * and text is the data being protected
         */

        /* start out by storing key in pads */
        memset(k_ipad, 0, sizeof k_ipad);
        memset(k_opad, 0, sizeof k_opad);
        memmove(k_ipad, key, key_len);
        memmove(k_opad, key, key_len);

        /* XOR key with ipad and opad values */
        for (i=0; i<64; i++) {
                k_ipad[i] ^= 0x36;
                k_opad[i] ^= 0x5c;
        }
        /*
         * perform inner MD5
         */
        esp_rom_md5_init(&context);                   /* init context for 1st
                                              * pass */
        esp_rom_md5_update(&context, k_ipad, 64);     /* start with inner pad */
        esp_rom_md5_update(&context, text, text_len); /* then text of datagram */
        esp_rom_md5_final(digest, &context);          /* finish up 1st pass */
        /*
         * perform outer MD5
         */
        esp_rom_md5_init(&context);                   /* init context for 2nd
                                              * pass */
        esp_rom_md5_update(&context, k_opad, 64);     /* start with outer pad */
        esp_rom_md5_update(&context, digest, 16);     /* then results of 1st
                                              * hash */
        esp_rom_md5_final(digest, &context);          /* finish up 2nd pass */
}
