/**
 * @file md5.h
 * @The header file of md5.
 * @author Jiewei Wei
 * @mail weijieweijerry@163.com
 * @github https://github.com/JieweiWei
 * @data Oct 19 2014
 *
 */

#ifndef MD5_H
#define MD5_H

#include <cstdint>
#include <cstring>

namespace uftp {

class MD5 {
 public:
  /* Construct a MD5 object with a string. */
  MD5();

  /* Construct a MD5 object with a string. */
  MD5(const char* message);

  /* Initialization the md5 object, processing another message block,
   * and updating the context.*/
  void Update(const uint8_t* input, size_t len);

  /* Generate md5 digest. */
  const uint8_t* GetDigest();

  /* Convert digest to string value */
  void ToString(char str[33]);

 private:
  /* MD5 basic transformation. Transforms state based on block. */
  void Transform(const uint8_t* block);

  /* Encodes input (usigned long) into output (byte). */
  static void Encode(const uint32_t* input, uint8_t* output, size_t length);

  /* Decodes input (byte) into output (usigned long). */
  static void Decode(const uint8_t* input, uint32_t* output, size_t length);

 private:
  /* Flag for mark whether calculate finished. */
  bool finished;

  /* state (ABCD). */
  uint32_t state[4]{};

  /* number of bits, low-order word first. */
  uint32_t count[2]{};

  /* input buffer. */
  uint8_t buffer[64]{};

  /* message digest. */
  uint8_t digest[16]{};

  /* padding for calculate. */
  static const uint8_t PADDING[64];
};

}  // namespace uftp

#endif  // MD5_H
