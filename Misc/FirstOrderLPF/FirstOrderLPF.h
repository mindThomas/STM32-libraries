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
 
#ifndef MISC_FIRSTORDERLPF_H
#define MISC_FIRSTORDERLPF_H

class FirstOrderLPF
{
	public:
		FirstOrderLPF(float Ts, float tau);
		~FirstOrderLPF();
		
		float Filter(float input);
		void Reset(void);
		void ChangeTimeconstant(float tau);

	private:
		const float _Ts;       // Sampling Time
		float _tau;            // Filter time constant

		float _coeff_b = 0.0;  // IIR filter coefficient (nominator polynomial)
		float _coeff_a = 0.0;  // IIR filter coefficient (denominator polynomial)
		
		float _lpfOld = 0.0;   // Holds previous sample output value
		float _inputOld = 0.0; // Holds previous sample output value
};
	
	
#endif
