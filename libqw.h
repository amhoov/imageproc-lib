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

#ifndef _libqw_h_
#define _libqw_h_
 
 /*****************************************************************************
* Function Name : _Q16abs
* Description   : Find absolute value of _Q16
* Parameters    : _Q16 representation of number to find absolute value of
* Return Value  : Absolute value of argument
*****************************************************************************/
_Q16 _Q16abs(_Q16);
 
 /*****************************************************************************
* Function Name : _Q16mult
* Description   : Calculates signed product of two _Q16 numbers
* Parameters    : _Q16 multiplicand A, _Q16 multiplicand B
* Return Value  : Product of A and B with saturation
*****************************************************************************/
_Q16 _Q16mult(_Q16, _Q16);
 
 /*****************************************************************************
* Function Name : _Q16sign
* Description   : Find sign of a _Q16 number
* Parameters    : _Q16 number to find sign of
* Return Value  : -1 if negative, 1 if positive, 0 if zero
*****************************************************************************/
char _Q16sign(_Q16);
 
 /*****************************************************************************
* Function Name : _IQ16div
* Description   : Divides two _Q16 numbers 
* Parameters    : _Q16 dividend, _Q16 divisor
* Return Value  : Quotient of arguments
*****************************************************************************/
_Q16 _IQ16div(_Q16, _Q16);
 
 /*****************************************************************************
* Function Name : _uIQ16div
* Description   : Divides two unsigned _Q16 numbers
* Parameters    : _Q16 dividend, _Q16 divisor
* Return Value  : Quotient of arguments
*****************************************************************************/
_Q16 _uIQ16div(_Q16, _Q16);
 
 /*****************************************************************************
* Function Name : _Q16inttoi
* Description   : Converts an integer into a _Q16
* Parameters    : int to convert
* Return Value  : _Q16 representation
*****************************************************************************/
_Q16 _Q16inttoi(int);
 
 /*****************************************************************************
* Function Name : _itointQ16
* Description   : Converts _Q16 into integer
* Parameters    : _Q16 to convert
* Return Value  : int representation with rounding
*****************************************************************************/
int _itointQ16(_Q16);

#endif
