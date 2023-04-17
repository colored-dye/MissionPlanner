namespace SimpleExample
{
    using System;
    using System.Runtime.InteropServices;

    /// <summary>
    /// Defines the <see cref="GEC" />.
    /// </summary>
    internal class GEC
    {
        /// <summary>
        /// Defines the GEC_PT_LEN.
        /// </summary>
        public const int GEC_PT_LEN = 64;

        /// <summary>
        /// Defines the GEC_CT_LEN.
        /// </summary>
        public const int GEC_CT_LEN = GEC_PT_LEN + 4 + 12;

        /// <summary>
        /// Defines the GEC_CT_FRAME_LEN.
        /// </summary>
        public const int GEC_CT_FRAME_LEN = GEC_CT_LEN + 1 + 1;

        /// <summary>
        /// Defines the GEC_CT_FRAME_MAGIC.
        /// </summary>
        public const byte GEC_CT_FRAME_MAGIC = 0x7e;

        /// <summary>
        /// Defines the GEC_CT_FRAME_TAG.
        /// </summary>
        public const byte GEC_CT_FRAME_TAG = 0;

        /// <summary>
        /// Defines the _GEC_SALT_LEN.
        /// </summary>
        public const int _GEC_SALT_LEN = 8;

        /// <summary>
        /// Defines the _GEC_SYM_CIPHER_KEY_LEN.
        /// </summary>
        public const int _GEC_SYM_CIPHER_KEY_LEN = 16;

        /// <summary>
        /// Defines the GEC_RAW_KEY_LEN.
        /// </summary>
        public const int GEC_RAW_KEY_LEN = _GEC_SALT_LEN + _GEC_SYM_CIPHER_KEY_LEN;

        /// <summary>
        /// Defines the GEC_SYM_KEY_LEN.
        /// </summary>
        private const int GEC_SYM_KEY_LEN = 3619312;

        /// <summary>
        /// Defines the sym_key_chan1.
        /// </summary>
        public Gec_sym_key sym_key_chan1;

        /// <summary>
        /// Defines the sym_key_chan2.
        /// </summary>
        public Gec_sym_key sym_key_chan2;

        /// <summary>
        /// Raw key, 24 bytes.
        /// Feed to `gec_init_sym_key_conf_auth()` function to get a gec_sym_key........
        /// </summary>
        public static readonly byte[] raw_key_1 = new byte[]
        {
            0xCB, 0x28, 0x4A, 0xD9, 0x1E, 0x85, 0x78, 0xB1,
            0x77, 0x6E, 0x9B, 0x98, 0x32, 0xEF, 0x11, 0xB0,
            0xBC, 0xA8, 0xCF, 0xD6, 0x29, 0x98, 0xDA, 0x15,
        };

        /// <summary>
        /// Defines the raw_key_2.
        /// </summary>
        public static readonly byte[] raw_key_2 = new byte[]
        {
            0x43, 0x82, 0xC5, 0xAC, 0x4C, 0xB9, 0x58, 0xC5,
            0x57, 0x0A, 0x4E, 0x30, 0xCC, 0xED, 0xFE, 0xF7,
            0x76, 0xF7, 0xC7, 0x75, 0x0C, 0x53, 0xA9, 0xE5,
        };

        /// <summary>
        /// Defines the <see cref="Gec_sym_key" />.
        /// </summary>
        public class Gec_sym_key
        {
            /// <summary>
            /// Defines the byte_array.
            /// </summary>
            internal byte[] byte_array;

            /// <summary>
            /// Initializes a new instance of the <see cref="Gec_sym_key"/> class.
            /// </summary>
            public Gec_sym_key()
            {
                byte_array = new byte[GEC_SYM_KEY_LEN];
            }
        }

        /// <summary>
        /// Defines the <see cref="Gec_raw_key" />.
        /// </summary>
        public class Gec_raw_key
        {
            /// <summary>
            /// Defines the byte_array.
            /// </summary>
            internal byte[] byte_array;

            /// <summary>
            /// Initializes a new instance of the <see cref="Gec_raw_key"/> class.
            /// </summary>
            public Gec_raw_key()
            {
                byte_array = new byte[GEC_RAW_KEY_LEN];
            }

            /// <summary>
            /// Initializes a new instance of the <see cref="Gec_raw_key"/> class.
            /// </summary>
            /// <param name="b">The b<see cref="byte[]"/>.</param>
            public Gec_raw_key(byte[] b)
            {
                byte_array = (byte[])b.Clone();
            }
        }

        /// <summary>
        /// Defines the <see cref="Gec_key_material" />.
        /// </summary>
        public class Gec_key_material
        {
            /// <summary>
            /// Defines the byte_array.
            /// </summary>
            internal byte[] byte_array;

            /// <summary>
            /// Initializes a new instance of the <see cref="Gec_key_material"/> class.
            /// </summary>
            public Gec_key_material()
            {
                byte_array = new byte[2 * GEC_RAW_KEY_LEN];
            }

            /// <summary>
            /// Initializes a new instance of the <see cref="Gec_key_material"/> class.
            /// </summary>
            /// <param name="b">The b<see cref="byte[]"/>.</param>
            public Gec_key_material(byte[] b)
            {
                b.CopyTo(byte_array, 0);
            }
        }

        /// <summary>
        /// Defines the <see cref="Gec_plaintext" />.
        /// </summary>
        public class Gec_plaintext
        {
            /// <summary>
            /// Defines the byte_array.
            /// </summary>
            public byte[] byte_array;

            /// <summary>
            /// Initializes a new instance of the <see cref="Gec_plaintext"/> class.
            /// </summary>
            public Gec_plaintext()
            {
                byte_array = new byte[GEC_PT_LEN];
            }

            /// <summary>
            /// The print.
            /// </summary>
            public void print()
            {
                Console.WriteLine("==========================");
                Console.WriteLine("Plaintext");
                for (var i = 0; i < GEC_PT_LEN; i++)
                {
                    string s = String.Format("{0:D3} ", this.byte_array[i]);
                    Console.Write(s);
                    if ((i + 1) % 10 == 0)
                    {
                        Console.Write('\n');
                    }
                }
                Console.WriteLine("==========================");
            }
        }

        /// <summary>
        /// Defines the <see cref="Gec_ciphertext" />.
        /// </summary>
        public class Gec_ciphertext
        {
            /// <summary>
            /// Defines the byte_array.
            /// </summary>
            public byte[] byte_array;

            /// <summary>
            /// Initializes a new instance of the <see cref="Gec_ciphertext"/> class.
            /// </summary>
            public Gec_ciphertext()
            {
                byte_array = new byte[GEC_CT_LEN];
            }

            public Gec_ciphertext(byte[] ct): this()
            {
                Array.ConstrainedCopy(ct, 0, byte_array, 0, GEC_CT_LEN);
            }
        }

        /// <summary>
        /// The gec_init_sym_key_conf_auth.
        /// </summary>
        /// <param name="k">The k<see cref="Gec_sym_key"/>.</param>
        /// <param name="rawkey">The rawkey<see cref="Gec_raw_key"/>.</param>
        [DllImport("GEC_base.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern void gec_init_sym_key_conf_auth([Out] byte[] k, [In] byte[] rawkey);

        /// <summary>
        /// The gec_key_material_to_2_channels.
        /// </summary>
        /// <param name="chan1">The chan1<see cref="Gec_sym_key"/>.</param>
        /// <param name="chan2">The chan2<see cref="Gec_sym_key"/>.</param>
        /// <param name="key_material">The key_material<see cref="Gec_key_material"/>.</param>
        [DllImport("GEC_base.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern void gec_key_material_to_2_channels([Out] byte[] chan1, [Out] byte[] chan2, [In] byte[] key_material);

        /// <summary>
        /// The gec_encrypt.
        /// </summary>
        /// <param name="k">The k<see cref="Gec_sym_key"/>.</param>
        /// <param name="pt">The pt<see cref="byte[]"/>.</param>
        /// <param name="ct">The ct<see cref="byte[]"/>.</param>
        /// <returns>The <see cref="int"/>.</returns>
        [DllImport("GEC_base.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern int gec_encrypt([In] byte[] k, [In] byte[] pt, [Out] byte[] ct);

        /// <summary>
        /// The gec_decrypt.
        /// </summary>
        /// <param name="k">The k<see cref="Gec_sym_key"/>.</param>
        /// <param name="ct">The ct<see cref="byte[]"/>.</param>
        /// <param name="pt">The pt<see cref="byte[]"/>.</param>
        /// <returns>The <see cref="int"/>.</returns>
        [DllImport("GEC_base.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern int gec_decrypt([In] byte[] k, [In] byte[] ct, [Out] byte[] pt);

        /// <summary>
        /// Initializes a new instance of the <see cref="GEC"/> class.
        /// </summary>
        public GEC()
        {
            sym_key_chan1 = new Gec_sym_key();
            sym_key_chan2 = new Gec_sym_key();
            Gec_raw_key raw_key1 = new Gec_raw_key(raw_key_1);
            Gec_raw_key raw_key2 = new Gec_raw_key(raw_key_2);
            gec_init_sym_key_conf_auth(sym_key_chan1.byte_array, raw_key1.byte_array);
            gec_init_sym_key_conf_auth(sym_key_chan2.byte_array, raw_key2.byte_array);
        }

        /// <summary>
        /// The encrypt.
        /// </summary>
        /// <param name="chan">The chan<see cref="int"/>.</param>
        /// <param name="pt">The pt<see cref="Gec_plaintext"/>.</param>
        /// <param name="ct">The ct<see cref="Gec_ciphertext"/>.</param>
        /// <returns>The <see cref="bool"/>.</returns>
        public bool encrypt(int chan, Gec_plaintext pt, Gec_ciphertext ct)
        {
            if (chan == 1)
            {
                return gec_encrypt(sym_key_chan1.byte_array, pt.byte_array, ct.byte_array) == 0;
            }
            else
            {
                return gec_encrypt(sym_key_chan2.byte_array, pt.byte_array, ct.byte_array) == 0;
            }
        }

        /// <summary>
        /// The decrypt.
        /// </summary>
        /// <param name="chan">The chan<see cref="int"/>.</param>
        /// <param name="ct">The ct<see cref="Gec_ciphertext"/>.</param>
        /// <param name="pt">The pt<see cref="Gec_plaintext"/>.</param>
        /// <returns>The <see cref="bool"/>.</returns>
        public bool decrypt(int chan, Gec_ciphertext ct, Gec_plaintext pt)
        {
            if (chan == 1)
            {
                return gec_decrypt(sym_key_chan1.byte_array, ct.byte_array, pt.byte_array) == 0;
            }
            else
            {
                return gec_decrypt(sym_key_chan2.byte_array, ct.byte_array, pt.byte_array) == 0;
            }
        }
    }
}
