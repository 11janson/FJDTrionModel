using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;


namespace Encrypt
{
    public class EncryptUtils
    {
        /// <summary>
        /// <
        /// </summary>
        public EncryptUtils()
        {

        }

        /// <summary>
        /// Aes加密
        /// </summary>
        /// <param name="sData"></param>
        /// <param name="strKey"></param>
        /// <returns></returns>
        public static string AesEncrypt(string sData, string strKey)
        {
            try
            {
                byte[] keyArray = UTF8Encoding.UTF8.GetBytes(strKey);
                byte[] toEncryptArray = UTF8Encoding.UTF8.GetBytes(sData);

                RijndaelManaged rDel = new RijndaelManaged();
                rDel.Key = keyArray;
                rDel.IV = keyArray;
                rDel.Mode = CipherMode.CBC;
                rDel.Padding = PaddingMode.PKCS7;
                ICryptoTransform cTransform = rDel.CreateEncryptor();
                byte[] resultArray = cTransform.TransformFinalBlock(toEncryptArray, 0, toEncryptArray.Length);
                return Convert.ToBase64String(resultArray, 0, resultArray.Length);
            }
            catch
            {

            }

            return string.Empty;
        }

        /// <summary>
        /// Aes解密
        /// </summary>
        /// <param name="sData"></param>
        /// <param name="strKey"></param>
        /// <returns></returns>
        public static string AesDecrypt(string sData, string strKey)
        {
            try
            {
                byte[] keyArray = UTF8Encoding.UTF8.GetBytes(strKey);
                byte[] toEncryptArray = Convert.FromBase64String(sData);

                RijndaelManaged rDel = new RijndaelManaged();
                rDel.Key = keyArray;
                rDel.IV = keyArray;
                rDel.Mode = CipherMode.CBC;
                rDel.Padding = PaddingMode.PKCS7;
                ICryptoTransform cTransform = rDel.CreateDecryptor();
                byte[] resultArray = cTransform.TransformFinalBlock(toEncryptArray, 0, toEncryptArray.Length);
                return UTF8Encoding.UTF8.GetString(resultArray);
            }
            catch
            {

            }

            return string.Empty;
        }

        public static string DesEncrypt(string sData, string strKey)
        {
           

            return string.Empty;
        }

        public static string DesDecrypt(string sData, string strKey)
        {
           

            return string.Empty;
        }

        public static string getMd5(string sData)
        {
            try
            {
                MD5CryptoServiceProvider md5 = new MD5CryptoServiceProvider();
                byte[] bytValue, bytHash;
                bytValue = System.Text.Encoding.UTF8.GetBytes(sData);
                bytHash = md5.ComputeHash(bytValue);
                md5.Clear();
                string sTemp = "";
                for (int i = 0; i < bytHash.Length; i++)
                {
                    sTemp += bytHash[i].ToString("X").PadLeft(2, '0');
                }
               return sTemp.ToLower();
            }
            catch 
            {
             
            }

            return string.Empty;
        }







    }
}
