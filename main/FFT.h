/*
 * FFT.h
 *
 *  Created on: 10 dic. 2022
 *      Author: Ramón
 */

#ifndef MAIN_FFT_H_
#define MAIN_FFT_H_

typedef float ftab_type[MAX_BINS * 2];

__attribute__((aligned(16)))
ftab_type twiddle_factors;				// complex (cos,sin)
// Window coefficients
__attribute__((aligned(16)))
float wind[MAX_BINS];					// real only
__attribute__((aligned(16)))
ftab_type fir_coeff;					// complex

void FFT_init(int N)
{
	dsps_fft2r_init_fc32(twiddle_factors, CONFIG_DSP_MAX_FFT_SIZE);	// init FFT tables
	dsps_gen_w_r2_fc32(twiddle_factors, N*2);	// init factor cos/sin
    dsps_wind_hann_f32(wind, N);				// Generate hann window
}

void FFT(float *data, int N)	// FFT forward
{
    dsps_fft2r_fc32(data, N);
    dsps_bit_rev_fc32(data, N);			// Bit reverse
}

void rFFT(float *data, int N, int method)	// FFT reverse
{
	if (method==1) {
		}
	else if (method==2)	{
		}
	else if (method==3)	{
		for (int i=0; i<N;i++)	{
			float auxf = data[2*i];
			data[2*i] = data[2*i+1];
			data[2*i+1] = auxf;
			}
	    dsps_fft2r_fc32(data, N);
		for (int i=0; i<N;i++)	{
			float auxf = data[2*i]/N;
			data[2*i] = data[2*i+1]/N;
			data[2*i+1] = auxf;
			}
		}
	else if (method==4)	{
	    for (int i=0;i<N;i++)	{		// conjugate
	    	//data[2*i] = data[2*i];
	    	data[2*i+1] = -data[2*i+1];
    		}
	    dsps_fft2r_fc32(data, N);					// FFT forward
		for (int i=0;i<N;i++)	{		// re conjugate
			data[2*i] = data[2*i] / N;
			data[2*i+1] = -data[2*i+1]/ N;
			}
		}
    dsps_bit_rev_fc32(data, N);			// Bit reverse
}



#endif /* MAIN_FFT_H_ */
