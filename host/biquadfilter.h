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

class biquadLPF
{
public:
  biquadLPF (float filtf, float q, float sampf);
  float update (float input);
  float reset (float value);
private:
  // precompute the coefficients
  float m_b0 = 1.0f;
  float m_b1 = 0.0f;
  float m_b2 = 0.0f;
  float m_a1 = 0.0f;
  float m_a2 = 0.0f;
  float m_d1 = 0.0f;
  float m_d2 = 0.0f;
};
