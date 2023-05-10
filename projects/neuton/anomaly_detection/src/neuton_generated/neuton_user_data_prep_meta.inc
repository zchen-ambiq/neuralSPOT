/* 2023-05-09T22:59:06Z */

/* ----------------------------------------------------------------------
Copyright (c) 2022-2023 Neuton.AI, Inc.
*
The source code and its binary form are being made available on the following terms:
Redistribution, use, and modification are permitted provided that the following
conditions are met:
*
1. Redistributions of source code and/or its binary form must retain the above copyright notice,
* this list of conditions (and the disclaimer) either in the body of the source code or in
* the documentation and/or other materials provided with the distribution of the binary form, as
applicable.
*
2. The name of the copyright holder may not be used to endorse or promote products derived from this
* source code or its binary form without specific prior written permission of Neuton.AI, Inc.
*
3. Disclaimer. THIS SOURCE CODE AND ITS BINARY FORM ARE PROVIDED BY THE COPYRIGHT HOLDER "AS IS".
* THE COPYRIGHT HOLDER HEREBY DISCLAIMS ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
* BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE HELD LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS
* OF THIRD PARTIES; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOURCE CODE OR ITS BINARY FORM, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
---------------------------------------------------------------------- */

/** Defines input(also used for LAG) features MIN scaling factor
 */
static const neuton_input_t INPUT_FEATURES_SCALE_MIN[] = { 0 };

/** Defines input(also used for LAG) features MAX scaling factor
 */
static const neuton_input_t INPUT_FEATURES_SCALE_MAX[] = { 0 };

/** Defines which unique input features is used for LAG features processing,
 *  one bit for one unique feature, starting from LSB
 */
static const neuton_u8_t INPUT_FEATURES_USED_FOR_LAGS_MASK[] = { 0x00 };

/** Defines DSP feature calculation masks used as neuton_dsp_features_mask_t,
 *  32 bit for one unique feature, @ref neuton_dsp_features_mask_t to see bitmask
 */
static const neuton_u32_t DSP_FEATURES_CALCULATION_MASK[] = {
	1091764224, -1013383168, -140574720, -158400512 };

/** Defines DSP feature calculation arguments used by DSP pipeline,
 *  32 bit for one feature argument
 */
static const neuton_input_t DSP_FEATURES_CALCULATION_ARGS[] = { 4, 4, 4, 4 };

/** Defines DSP features MAX scaling factor
 */
static const neuton_feature_t DSP_FEATURES_SCALE_MIN[] = {
	20, 20, 0, 20, 20, -8, 0.51760149002075195, 0.01276562362909317, 0.015221797861158848,
	0.94538849592208862, 0.18897637724876404, 0.90924227237701416, 0.019527556374669075,
	-8, 0.082000002264976501, 0.17199999094009399, -0.50541400909423828, 0.03543785959482193,
	0.042742688208818436, 0.043420992791652679, 0.36220473051071167, 0.36220473051071167,
	0.085000000894069672, 0.16175000369548798, 0.03560936450958252, 0.048503927886486053,
	-5.8670001029968262, -0.10899999737739563, 0.050000004470348358, -0.34387493133544922,
	0.011627982370555401, 0.28346458077430725, 0, 0.021500006318092346, 0.04900001734495163,
	0.11164841055870056, 0.014181100763380527 };

/** Defines DSP features MIN scaling factor
 */
static const neuton_feature_t DSP_FEATURES_SCALE_MAX[] = {
	100, 100, 2.5, 100, 100, 0.96899998188018799, 1.4421483278274536, 1.9972096681594849,
	2.5022830963134766, 2.7515442371368408, 0.77952754497528076, 2.1941947937011719,
	2.7542209625244141, -0.078000001609325409, 7.995999813079834, 15.562000274658203,
	0.45989832282066345, 1.7804734706878662, 2.4769837856292725, 2.4998757839202881,
	0.79527556896209717, 0.79527556896209717, 6.8814992904663086, 14.157749176025391,
	1.7723987102508545, 2.724881649017334, -0.14800000190734863, 6.0859999656677246,
	9.6599998474121094, 0.24049219489097595, 1.6843562126159668, 0.75590550899505615,
	0.66929131746292114, 4.5499997138977051, 9.1452503204345703, 1.3436875343322754,
	2.0365588665008545 };

