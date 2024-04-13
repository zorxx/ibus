/* \copyright 2024 Zorxx Software. All rights reserved.
 * \license This file is released under the MIT License. See the LICENSE file for details.
 */
#ifndef _IBUS_HELPERS_H
#define _IBUS_HELPERS_H

#include <stdio.h>

#if defined(IBUS_DEBUG_ENABLE)
#define IBUSERR(...) fprintf(stderr, __VA_ARGS__)
#define IBUSDBG(...) fprintf(stderr, __VA_ARGS__)
#else
#define IBUSERR(...)
#define IBUSDBG(...)
#endif

#endif /* _IBUS_HELPERS_H */