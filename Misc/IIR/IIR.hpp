/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */
 
#ifndef MISC_IIR_H
#define MISC_IIR_H

#include "Debug.h"

template <int ORDER>
class IIR
{
		public:

		IIR(const float coeff_a[], const float coeff_b[])
		{
			for (int i = 0; i < ORDER+1; i++) {
				a[i] = coeff_a[i];
				b[i] = coeff_b[i];
			}
			for (int i = 0; i < ORDER; i++) {
				u_old[i] = 0;
				y_old[i] = 0;
			}
		}

		void Initialize(const float coeff_a[], const float coeff_b[])
		{
			for (int i = 0; i < ORDER+1; i++) {
				a[i] = coeff_a[i];
				b[i] = coeff_b[i];
			}
			for (int i = 0; i < ORDER; i++) {
				u_old[i] = 0;
				y_old[i] = 0;
			}
		}

		float Filter(float input)
		{
			// a[0]*y[k] + a[1]*y[k-1] + ... = b[0]*u[k] + b[1]*u[k-1] + ...
			float output;
			float ay = 0;
			float bu = 0;

			bu = b[0] * input;
			for (int i = ORDER; i > 0; i--) {
				bu += b[i] * u_old[i-1];
				ay += a[i] * y_old[i-1];

				if (i > 1) {
					u_old[i-1] = u_old[i-2];
					y_old[i-1] = y_old[i-2];
				}
			}

			output = 1/a[0] * (bu - ay);

			u_old[0] = input;
			y_old[0] = output;

			return output;
		}

		private:
			float a[ORDER+1];
			float b[ORDER+1];

			float u_old[ORDER];
			float y_old[ORDER];
};
	
	
#endif
