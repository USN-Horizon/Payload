#pragma once

// Cross-platform replacement for the BSD/glibc <endian.h> byte-swap macros.
//
// Windows (MSVC/MinGW) ships no <endian.h>, and the BSD/Apple variants live in
// different headers. Qt's <QtEndian> provides the same conversions portably, so
// we route the htoleNN / leNNtoh macros used by msgmap.hpp through it.

#include <QtEndian>

#ifndef htole16
#define htole16(x) qToLittleEndian<quint16>(static_cast<quint16>(x))
#endif
#ifndef htole32
#define htole32(x) qToLittleEndian<quint32>(static_cast<quint32>(x))
#endif
#ifndef htole64
#define htole64(x) qToLittleEndian<quint64>(static_cast<quint64>(x))
#endif

#ifndef le16toh
#define le16toh(x) qFromLittleEndian<quint16>(static_cast<quint16>(x))
#endif
#ifndef le32toh
#define le32toh(x) qFromLittleEndian<quint32>(static_cast<quint32>(x))
#endif
#ifndef le64toh
#define le64toh(x) qFromLittleEndian<quint64>(static_cast<quint64>(x))
#endif
