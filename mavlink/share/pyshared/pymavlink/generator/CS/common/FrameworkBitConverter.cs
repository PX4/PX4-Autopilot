using System;

namespace MavLink
{
    /// <summary>
    /// converter from byte[] => primitive CLR types
    /// delegates to the .Net framework bitconverter for speed, and to avoid using unsafe pointer 
    /// casting for Silverlight.
    /// </summary>
    internal class FrameworkBitConverter
    {
        private bool _shouldReverse = false;

        public void SetDataIsLittleEndian(bool islittle)
        {
            _shouldReverse = islittle == !BitConverter.IsLittleEndian;
        }

        public UInt16 ToUInt16(byte[] value, int startIndex)
        {
            if (_shouldReverse)
            {
                var bytes = new[] {value[startIndex + 1], value[startIndex]};
                return BitConverter.ToUInt16(bytes,0);
            }
            return BitConverter.ToUInt16(value, startIndex);
        }

        public Int16 ToInt16(byte[] value, int startIndex)
        {
            if (_shouldReverse)
            {
                var bytes = new[] { value[startIndex + 1], value[startIndex] };
                return BitConverter.ToInt16(bytes, 0);
            }
            return BitConverter.ToInt16(value, startIndex);
        }

        public sbyte ToInt8(byte[] value, int startIndex)
        {
            return unchecked((sbyte)value[startIndex]);
        }

        public Int32 ToInt32(byte[] value, int startIndex)
        {
            if (_shouldReverse)
            {
                var bytes = new byte[4];
                Array.Copy(value,startIndex,bytes,0,4);
                Array.Reverse(bytes);
                return BitConverter.ToInt32(bytes, 0);
            }
            return BitConverter.ToInt32(value, startIndex);
        }

        public UInt32 ToUInt32(byte[] value, int startIndex)
        {
            if (_shouldReverse)
            {
                var bytes = new byte[4];
                Array.Copy(value, startIndex, bytes, 0, 4);
                Array.Reverse(bytes);
                return BitConverter.ToUInt32(bytes, 0);
            }
            return BitConverter.ToUInt32(value, startIndex);
        }

        public UInt64 ToUInt64(byte[] value, int startIndex)
        {
            if (_shouldReverse)
            {
                var bytes = new byte[8];
                Array.Copy(value, startIndex, bytes, 0, bytes.Length);
                Array.Reverse(bytes);
                return BitConverter.ToUInt64(bytes, 0);
            }
            return BitConverter.ToUInt64(value, startIndex);
        }

        public Int64 ToInt64(byte[] value, int startIndex)
        {
            if (_shouldReverse)
            {
                var bytes = new byte[8];
                Array.Copy(value, startIndex, bytes, 0, bytes.Length);
                Array.Reverse(bytes);
                return BitConverter.ToInt64(bytes, 0);
            }
            return BitConverter.ToInt64(value, startIndex);
        }

        public Single ToSingle(byte[] value, int startIndex)
        {
            if (_shouldReverse)
            {
                var bytes = new byte[4];
                Array.Copy(value, startIndex, bytes, 0, bytes.Length);
                Array.Reverse(bytes);
                return BitConverter.ToSingle(bytes, 0);
            }
            return BitConverter.ToSingle(value, startIndex);
        }

        public Double ToDouble(byte[] value, int startIndex)
        {
            if (_shouldReverse)
            {
                var bytes = new byte[8];
                Array.Copy(value, startIndex, bytes, 0, bytes.Length);
                Array.Reverse(bytes);
                return BitConverter.ToDouble(bytes, 0);
            }
            return BitConverter.ToDouble(value, startIndex);
        }

        public void GetBytes(Double value, byte[] dst, int offset)
        {
            var bytes =  BitConverter.GetBytes(value);
            if (_shouldReverse) Array.Reverse(bytes);
            Array.Copy(bytes, 0, dst, offset, bytes.Length);
            
        }
      
        public void GetBytes(Single value, byte[] dst, int offset)
        {
            var bytes = BitConverter.GetBytes(value);
            if (_shouldReverse) Array.Reverse(bytes);

            Array.Copy(bytes, 0, dst, offset, bytes.Length);
        }

        public void GetBytes(UInt64 value, byte[] dst, int offset)
        {
            var bytes = BitConverter.GetBytes(value);
            if (_shouldReverse) Array.Reverse(bytes);

            Array.Copy(bytes, 0, dst, offset, bytes.Length);
        }

        public void GetBytes(Int64 value, byte[] dst, int offset)
        {
            var bytes = BitConverter.GetBytes(value);
            if (_shouldReverse) Array.Reverse(bytes);

            Array.Copy(bytes, 0, dst, offset, bytes.Length);
        }

        public void GetBytes(UInt32 value, byte[] dst, int offset)
        {
            var bytes = BitConverter.GetBytes(value);
            if (_shouldReverse) Array.Reverse(bytes);

            Array.Copy(bytes, 0, dst, offset, bytes.Length);
        }

        public void GetBytes(Int16 value, byte[] dst, int offset)
        {
            var bytes = BitConverter.GetBytes(value);
            if (_shouldReverse) Array.Reverse(bytes);

            Array.Copy(bytes, 0, dst, offset, bytes.Length);
        }

        public void GetBytes(Int32 value, byte[] dst, int offset)
        {
            var bytes = BitConverter.GetBytes(value);
            if (_shouldReverse) Array.Reverse(bytes);

            Array.Copy(bytes, 0, dst, offset, bytes.Length);
        }

        public void GetBytes(UInt16 value, byte[] dst, int offset)
        {
            var bytes = BitConverter.GetBytes(value);
            if (_shouldReverse) Array.Reverse(bytes);

            Array.Copy(bytes, 0, dst, offset, bytes.Length);
        }

        public byte[] GetBytes(sbyte value)
        {
            return new byte[1] 
                       { 
                           (byte)value, 
                       };
        }
    }
}