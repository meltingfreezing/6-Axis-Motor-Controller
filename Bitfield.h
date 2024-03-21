/* From https://blog.codef00.com/2014/12/06/portable-bitfields-using-c11/
 * No license defined.
 */

#ifndef BITFIELD_H_
#define BITFIELD_H_

#include <stdint.h>
#include <stddef.h>
#include <type_traits>
#include <typeinfo>
#include <cmath>
#include <array>
#include <limits>


// using std::uint8_t;
// using std::uint16_t;
// using std::uint32_t;
// using std::uint64_t;

// This library has been reported to produce undefined behavior.
// Disabling GCC optimizations seem to fix the problem.
#ifndef ESP_PLATFORM
#pragma GCC push_options
#pragma GCC optimize ("O0")
#endif

// Using macros to define missing bit operations and the constrain function.
#define bitRead(value,  bit) (((value) >> (bit)) & 1ULL)
#define bitSet(value,   bit) ((value) |=  (1ULL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1ULL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value,  bit))

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
//#define radians(deg) ((deg)*DEG_TO_RAD)
//#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

namespace {

template <size_t LastBit>
struct MinimumTypeHelper {
    typedef
        typename std::conditional<LastBit == 0 , void,
        typename std::conditional<LastBit <= 8 , uint8_t,
        typename std::conditional<LastBit <= 16, uint16_t,
        typename std::conditional<LastBit <= 32, uint32_t,
        typename std::conditional<LastBit <= 64, uint64_t,
        void>::type>::type>::type>::type>::type type;
};

}

template <size_t Index, size_t Bits = 1>
class BitField {
private:
    enum {
        Mask = (1u << Bits) - 1u
    };

    typedef typename MinimumTypeHelper<Index + Bits>::type T;
public:
    template <class T2>
    BitField &operator=(T2 value) {
        value_ = (value_ & ~((T)Mask << Index)) | (((T)value & (T)Mask) << Index);
        return *this;
    }

    operator T() const             { return (value_ >> Index) & (T)Mask; }
    explicit operator bool() const { return value_ & ((T)Mask << Index); }
    BitField &operator++()         { return *this = *this + 1; }
    T operator++(int)              { T r = *this; ++*this; return r; }
    BitField &operator--()         { return *this = *this - 1; }
    T operator--(int)              { T r = *this; --*this; return r; }
    size_t size() const            { return Bits; }

private:
    T value_;
};


template <size_t Index>
class BitField<Index, 1> {
private:
    enum {
        Bits = 1,
        Mask = 0x01
    };

    typedef typename MinimumTypeHelper<Index + Bits>::type T;
public:
    BitField &operator=(bool value) {
        value_ = (value_ & ~((T)Mask << Index)) | ((T)value << Index);
        return *this;
    }

    explicit operator bool() const { return value_ & ((T)Mask << Index); }

private:
    T value_;
};

#ifndef ESP_PLATFORM
#pragma GCC pop_options
#endif

#endif
