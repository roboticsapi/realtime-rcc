#ifndef BASE64_H_
#define BASE64_H_

#include <string>
#include <sstream>
#include <memory>

namespace base64
{

const char base64chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
const char base64pad = '=';

/**
 * @brief Encode a buffer on a string base64 encoded
 *
 * Base 64 encoding converts 3 bytes into 4 encoded ASCII
 * characters.
 * If the encoded string lenght is not multiple of 4
 * the padding character "=" is added for fill the buffer.
 * If the buf_size is 0 an empty string was returned.
 *
 * @param buf_ptr pointer to the source buffer
 * @param buf_size size of the buffer
 *
 * @return a string base64 encoded
 *
 * @see http://en.wikipedia.org/wiki/Base64
 */
std::string encode(unsigned char* buf_ptr, size_t buf_size)
{
    // If the buf_size is 0 return an empty string
    if (buf_size == 0)
        return "";

    unsigned int n = 0;
    unsigned char n0, n1, n2, n3;
    std::stringstream ss_result;

    // Iterate over the buffer
    for (size_t x = 0; x < buf_size; x += 3)
    {
        // Get 3 bytes of 8 bit from the buffer
        n = buf_ptr[x] << 16;

        if ((x + 1) < buf_size)
        {
            n += buf_ptr[x + 1] << 8;

            if ((x + 2) < buf_size)
            {
                n += buf_ptr[x + 2];
            }
        }

        // Transform on 4 bytes of 6 bit
        n0 = (unsigned char)(n >> 18) & 63;
        n1 = (unsigned char)(n >> 12) & 63;
        n2 = (unsigned char)(n >> 6) & 63;
        n3 = (unsigned char)n & 63;

        // Write first 2 bytes into the stream
        ss_result << base64chars[n0];
        ss_result << base64chars[n1];

        // Test if only 2 bytes
        if ((x + 1) < buf_size)
        {
            // Write 3rd into the stream
            ss_result << base64chars[n2];

            // Test if only 3 bytes
            if ((x + 2) < buf_size)
            {
                // Write 4th into the stream
                ss_result << base64chars[n3];
            }
        }
    }

    // Add padding character for fill the output string
    for (int pad_count = 0; pad_count < ss_result.str().size() % 4; pad_count++)
    {
        ss_result << base64pad;
    }

    return ss_result.str();
}

/*!
 * @brief Compute size of decoded stream
 *
 * @param encoded_size size of the encoded stream
 * @return the size of the decoded stream
 */
size_t decoded_size(size_t encoded_size)
{
    return (encoded_size + 3) / 4 * 3;
}

/**
 * @brief Decode a string base64 encoded and return a raw buffer
 *
 * The function allocate a buffer with size:
 * @code (text.size() + 3) / 4 * 3
 * and return the pointer. The caller has the ownership of the
 * pointer.
 * The source string lenght must be multiple of 4.
 * Use is_base_64 for test if a string is Base64 well formed.
 *
 * @param text the base64 string encoded
 *
 * @return the pointer to the decoded buffer
 */
std::auto_ptr< unsigned char > decode(const std::string& text)
{
    // Create the decode matrix
    unsigned char decode_base64chars[256];
    memset (decode_base64chars, 0, 256);

    for (int i = 0; i <= 63; i++)
    {
        decode_base64chars[(int)base64chars[(int)i]] = i;
    }

    // Allocate output buffer
    unsigned char* buf_ptr = new unsigned char[decoded_size(text.size())];

    // Start decode source string
    size_t buf_idx = 0;
    for (size_t i = 0; i < text.size(); i += 4, buf_idx += 3)
    {
        // Put first base64 char on the first output-byte
        buf_ptr[buf_idx] = decode_base64chars[(int)text[i]] << 2;

        // Test for string end
        if ((i + 1 < text.size()) && (text[i + 1] != base64pad))
        {
            // Add to the first output-byte 2 bit of the second base64 char
            buf_ptr[buf_idx] += (decode_base64chars[(int)text[i + 1]] >> 4) & 0x3;

            // Put remaining 4 bit of the second base64 char on the second output-byte
            buf_ptr[buf_idx + 1] = decode_base64chars[(int)text[i + 1]] << 4;

            // Test for string end
            if ((i + 2 < text.size()) && (text[i + 2] != base64pad))
            {
                // Add to the second output-byte 4 bit of the third base64 char
                buf_ptr[buf_idx + 1] += (decode_base64chars[(int)text[i + 2]] >> 2) & 0xf;

                // Put remaining 2 bit of the third base64 char on the third output-byte
                buf_ptr[buf_idx + 2] = decode_base64chars[(int)text[i + 2]] << 6;

                // Test for string end
                if ((i + 1 < text.size()) && (text[i + 3] != base64pad))
                {
                    // Add to the third output-byte the forth base64 char
                    buf_ptr[buf_idx + 2] += decode_base64chars[(int)text[i + 3]];
                }
            }
        }
    }

    return std::auto_ptr<unsigned char>(buf_ptr);
}

/**
 * @brief Test if a string is well Base64 encoded
 *
 * @param text the string Base64 encoded
 *
 * @return true if the string is well formed
 */
bool is_base_64(const std::string& text)
{
    // Create the decode matrix
	unsigned char decode_base64chars[256];
    memset (decode_base64chars, 65, 256);

    for (unsigned char i = 0; i <= 63; i++)
    {
        decode_base64chars[(int)base64chars[(int)i]] = i;
    }

    // Add padding character
    decode_base64chars[(int)'='] = 64;

    // Test string characters
    for (size_t i = 0; i < text.size(); i++)
    {
        if (decode_base64chars[(int)text[i]] == 65)
        {
            // Invalid character found
            return false;
        }
        else if ((decode_base64chars[(int)text[i]] == 64) && (i < text.size() - 3))
        {
            // Padding character at invalid position
            return false;
        }
    }

    return true;
}

};

#endif /*BASE64_H_*/
