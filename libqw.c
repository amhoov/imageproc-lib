/*
 * Copyright (c) 2010, Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the University of California, Berkeley nor the names
 *   of its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Fixed point wrappper library
 *
 * by Humphrey Hu
 *
 * v.0.1
 *
 * Revisions:
 *  Humphrey Hu	      2011-06-29    Initial implementation
 *                      
 * Notes:
 *  
 */

#include <libq.h>

#include "libqw.h"
 
_Q16 _Q16abs(_Q16 q) {
    if(q < 0) {
		return _Q16neg(q);
	}
	return q;
}
 
char _Q16sign(_Q16 q) {
	if(q == 0) {
		return 0;
	} else if(q < 0) {
		return -1;
	} else {
		return 1;
	}
}
 
_Q16 _Q16mult(_Q16 multA, _Q16 multB) {
 
	int multSig = _Q16sign(multA)*_Q16sign(multB);
	
	_Q16 prod = _Q16mac(_Q16abs(multA), _Q16abs(multB), 0);
	if(multSig == -1) {
		prod = _Q16neg(prod);
	}
	return prod;
 
}
 
_Q16 _Q16inttoi(int i) {
 
	return (_Q16)(((long) i << 16));
 
}

int _itointQ16(_Q16 q) {
 
	return (int) (q >> 16);
 
}
