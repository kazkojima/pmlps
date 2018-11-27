// Copyright (C) 2018 kaz Kojima
//
// This file is part of PMLPS program.  This program is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the
// Free Software Foundation; either version 3, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program;
// see the files COPYING and EXCEPTION respectively.

#include "biquadfilter.h"

#include <cmath>

static const float float_pi = 3.14159265359;

biquadLPF::biquadLPF (float filtf, float q, float sampf)
{
  // setup variables
  const float omega = 2*float_pi*filtf/sampf;
  const float sn = sinf (omega);
  const float cs = cosf (omega);
  const float alpha = sn/(2*q);
  float b0, b1, b2, a0, a1, a2;
  // FILTER_NOTCH:
  // b0 =  1;
  // b1 = -2 * cs;
  // b2 =  1;
  b0 = (1 - cs)/2;
  b1 = 1 - cs;
  b2 = (1 - cs)/2;
  a0 =  1 + alpha;
  a1 = -2 * cs;
  a2 =  1 - alpha;

  // precompute the coefficients
  m_b0 = b0/a0;
  m_b1 = b1/a0;
  m_b2 = b2/a0;
  m_a1 = a1/a0;
  m_a2 = a2/a0;
  
  // zero initial samples
  m_d1 = m_d2 = 0;
}

// Computes a biquad_t filter on a sample
float biquadLPF::update (float input)
{
  const float result = m_b0*input + m_d1;
  m_d1 = m_b1*input - m_a1*result + m_d2;
  m_d2 = m_b2*input - m_a2*result;
  return result;
}

float biquadLPF::reset (float value)
{
  m_d1 = value - (value*m_b0);
  m_d2 = (m_b2 - m_a2)*value;
  return value;
}
