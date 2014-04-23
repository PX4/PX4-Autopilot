using System;
using System.Text;

namespace MavLink
{
    internal static class ByteArrayUtil
    {
#if MF_FRAMEWORK_VERSION_V4_1
        private static readonly MavBitConverter bitConverter = new MavBitConverter(); 
#else
        private static readonly FrameworkBitConverter bitConverter = new FrameworkBitConverter(); 
#endif
        
        public static byte[] ToChar(byte[] source, int sourceOffset, int size)
        {
            var bytes = new byte[size];

            for (int i = 0; i < size; i++)
                bytes[i] = source[i + sourceOffset];

            return bytes;
        }

        public static byte[] ToUInt8(byte[] source, int sourceOffset, int size)
        {
            var bytes = new byte[size];
            Array.Copy(source, sourceOffset, bytes, 0, size);
            return bytes;
        }

        public static sbyte[] ToInt8(byte[] source, int sourceOffset, int size)
        {
            var bytes = new sbyte[size];

            for (int i = 0; i < size; i++)
                bytes[i] = unchecked((sbyte)source[i + sourceOffset]);

            return bytes;
        }

        public static UInt16[] ToUInt16(byte[] source, int sourceOffset, int size)
        {
            var arr = new UInt16[size];
            for (int i = 0; i < size; i++)
                arr[i] = bitConverter.ToUInt16(source, sourceOffset + (i * sizeof (UInt16)));
            return arr;
        }

        public static Int16[] ToInt16(byte[] source, int sourceOffset, int size)
        {
            var arr = new Int16[size];
            for (int i = 0; i < size; i++)
                arr[i] = bitConverter.ToInt16(source, sourceOffset + (i * sizeof(Int16)));
            return arr;
        }

        public static UInt32[] ToUInt32(byte[] source, int sourceOffset, int size)
        {
            var arr = new UInt32[size];
            for (int i = 0; i < size; i++)
                arr[i] = bitConverter.ToUInt16(source, sourceOffset + (i * sizeof(UInt32)));
            return arr;
        }

        public static Int32[] ToInt32(byte[] source, int sourceOffset, int size)
        {
            var arr = new Int32[size];
            for (int i = 0; i < size; i++)
                arr[i] = bitConverter.ToInt16(source, sourceOffset + (i * sizeof(Int32)));
            return arr;
        }

        public static UInt64[] ToUInt64(byte[] source, int sourceOffset, int size)
        {
            var arr = new UInt64[size];
            for (int i = 0; i < size; i++)
                arr[i] = bitConverter.ToUInt16(source, sourceOffset + (i * sizeof(UInt64)));
            return arr;
        }

        public static Int64[] ToInt64(byte[] source, int sourceOffset, int size)
        {
            var arr = new Int64[size];
            for (int i = 0; i < size; i++)
                arr[i] = bitConverter.ToInt16(source, sourceOffset + (i * sizeof(Int64)));
            return arr;
        }

        public static Single[] ToSingle(byte[] source, int sourceOffset, int size)
        {
            var arr = new Single[size];
            for (int i = 0; i < size; i++)
                arr[i] = bitConverter.ToUInt16(source, sourceOffset + (i * sizeof(Single)));
            return arr;
        }

        public static Double[] ToDouble(byte[] source, int sourceOffset, int size)
        {
            var arr = new Double[size];
            for (int i = 0; i < size; i++)
                arr[i] = bitConverter.ToInt16(source, sourceOffset + (i * sizeof(Double)));
            return arr;
        }

        public static void ToByteArray(byte[] src, byte[] dst, int offset, int size)
        {
            int i;
            for (i = 0; i < src.Length; i++)
                dst[offset + i] = src[i];
            while (i++ < size)
                dst[offset + i] = 0;
        }

        public static void ToByteArray(sbyte[] src, byte[] dst, int offset, int size)
        {
            int i;
            for (i = 0; i < size && i<src.Length; i++)
                dst[offset + i] = (byte)src[i];
            while (i++ < size)
                dst[offset + i] = 0;
        }

        public static void ToByteArray(UInt16[] src, byte[] dst, int offset, int size)
        {
            for (int i = 0; i < size && i < src.Length; i++)
                bitConverter.GetBytes(src[i], dst, offset + (i*sizeof (UInt16)));
        }

        public static void ToByteArray(Int16[] src, byte[] dst, int offset, int size)
        {
            for (int i = 0; i < size && i < src.Length; i++)
                bitConverter.GetBytes(src[i], dst, offset + (i * sizeof(Int16)));
        }

        public static void ToByteArray(Int32[] src, byte[] dst, int offset, int size)
        {
            for (int i = 0; i < size && i < src.Length; i++)
                bitConverter.GetBytes(src[i], dst, offset + (i * sizeof(Int32)));
        }

        public static void ToByteArray(UInt32[] src, byte[] dst, int offset, int size)
        {
            for (int i = 0; i < size && i < src.Length; i++)
                bitConverter.GetBytes(src[i], dst, offset + (i * sizeof(UInt32)));
        }

        public static void ToByteArray(Single[] src, byte[] dst, int offset, int size)
        {
            for (int i = 0; i < size && i < src.Length; i++)
                bitConverter.GetBytes(src[i], dst, offset + (i * sizeof(Single)));
        }

        public static void ToByteArray(Double[] src, byte[] dst, int offset, int size)
        {
            for (int i = 0; i < size && i < src.Length; i++)
                bitConverter.GetBytes(src[i], dst, offset + (i * sizeof(Double)));
        }

        public static void ToByteArray(UInt64[] src, byte[] dst, int offset, int size)
        {
            for (int i = 0; i < size && i < src.Length; i++)
                bitConverter.GetBytes(src[i], dst, offset + (i * sizeof(UInt64)));
        }

        public static void ToByteArray(Int64[] src, byte[] dst, int offset, int size)
        {
            for (int i = 0; i < size && i < src.Length; i++)
                bitConverter.GetBytes(src[i], dst, offset + (i * sizeof(Int64)));
        }


       

        public static string ToString(sbyte[] sbytes)
        {
            var bytes = new byte[sbytes.Length];
            int i;
            for ( i = 0; i < bytes.Length && sbytes[i] != '\0'; i++)
                bytes[i] = (byte) sbytes[i];

            var bytesUntilNull = new byte[i];
            Array.Copy(bytes, bytesUntilNull, i);

            var encoding = new UTF8Encoding();

            return new string(encoding.GetChars(bytesUntilNull));
        }

        public static string ToString(byte[] bs)
        {
            int i;
            for (i = 0; i < bs.Length && bs[i] != '\0'; i++);
            var bytesUntilNull = new byte[i];
            Array.Copy(bs, bytesUntilNull, i);
            return new string(new UTF8Encoding().GetChars(bytesUntilNull));
        }
    }
}