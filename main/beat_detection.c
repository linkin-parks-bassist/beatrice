#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <esp_timer.h>
#include <assert.h>

#include "kiss_fft.h"

#include "led_control.h"
#include "buffers.h"
#include "mic.h"

#define PI 3.14159265

#define BEAT_THRESHOLD_INITIAL			1.4

#define ACTIVATION_THRESHOLD 			60.0
#define RUNNING_AVG_ENERGY_ADAPT_RATE 	0.99
#define RUNNING_AVG_WAIT_ADAPT_RATE   	0.8
#define RUNNING_AVG_SPECTRUM_ADAPT_RATE 0.5
#define BEAT_THRESHOLD_ADAPT_RATE     	0.95
#define BEAT_THRESHOLD_SENSITISE_RATE 	0.95
#define TIME_WEIGHT_EXPONENT 			2.0
#define TIME_WEIGHT_MAX					1.2
#define ANGLE_WEIGHT					1.2

#define AMP_BASS_WEIGHT					1.5
#define AMP_SNAP_WEIGHT					0.3
#define AMP_CRACK_WEIGHT				0.1

#define WAIT_NORMALISATION_FACTOR 		0.25

#define NORM_FACTOR 					(1.0 / (double)(1 << 23))

// This number is useful
static const double inv_erf_1 = 1.0/erf(1.0);

/*
 * This struct stores a number of parameters used to compare
 * the current frame to recent frames so as to judge whether
 * or not the current frame contains the onset of a beat. An
 * instance is allocated on the stack in the beat detection
 * task function and then passed as a pointer to the beat
 * detection function, which uses these values to compute its
 * result and also mutates the values of the struct according
 * to the given frame.
 */
typedef struct
{
	/* The threshold that, if the ratio of the energy of the current
	 * frame to the running average frame energy energy exceeds,
	 * it is judged to contain a beat */
	double beat_threshold;

	/* A running average of the energy of previous frames */
	double frame_energy_running_avg;
	
	// A running tally of how much time, in seconds, has passed since a beat was detected
	double seconds_since_last_beat;
	
	/* A running average of the time between beats; used to
	 * inhibit beat detection shortly after a beat, and disinhibit
	 * beat detection when then neat best is expected */
	double running_avg_wait;
	
	/* A running average of the Fourier transform of previous frames */
	double running_avg_spectrum[BUFFER_SIZE / 2];
} beat_detection_state;

// Some forward-declarations
int  beat_detected(beat_detection_state *bds,  const double *frame, const double *spectrum);
void perform_fft(kiss_fft_cfg kiss_fft_config, const double *frame,		  double *spectrum);

// A handy macro for squaring things
#define sqr(x) (x * x)

// A handy macro for the frequency in the xth index of the spectrum
#define FREQ(x)	((double)x * SAMPLE_RATE) / (double)BUFFER_SIZE;

// An array to hold values of the hamming window function for use in fft
static double hamming_window[BUFFER_SIZE];

// An array to hold the weights used for the amplitude energy calculation
double amplitude_freq_weights[BUFFER_SIZE / 2];
// An array to hold the weights used for weighting the spectral inner product
double angular_freq_weights[BUFFER_SIZE / 2];

// Write the values of the hamming window function
void init_hamming_window()
{
	for (int i = 0; i < BUFFER_SIZE; i++)
		hamming_window[i] = 0.54 - (0.46 * cos (2 * PI * ((double)i / (double)(BUFFER_SIZE - 1))));
}

// Produce weights for the weighted spectral inner product
void init_angular_freq_weights()
{
	double norm = 0;
	double frequency;
	
	for (int i = 0; i < BUFFER_SIZE / 2; i++)
	{
		frequency = FREQ(i);
		
		angular_freq_weights[i] = 2.0 * exp(-sqr(fabs(frequency - 80) /  25));
		norm += sqr(angular_freq_weights[i]);
		
		amplitude_freq_weights[i]  = AMP_BASS_WEIGHT  * exp(-sqr(fabs(frequency -   65) /  20));
        amplitude_freq_weights[i] += AMP_SNAP_WEIGHT  * exp(-sqr(fabs(frequency -  200) /  80));
        amplitude_freq_weights[i] += AMP_CRACK_WEIGHT * exp(-sqr(fabs(frequency - 1200) / 600));
	}
	
	if (norm == 0.0)
		return;
	
	for (int i = 0; i < BUFFER_SIZE / 2; i++)
		angular_freq_weights[i] /= norm;
}

// Initialise the given instance of the beat detection state struct
int init_beat_detection_state(beat_detection_state *bds)
{
	assert(bds != NULL);

	// Initialise with default values
	bds->beat_threshold 				= BEAT_THRESHOLD_INITIAL;
	
	// Assume 120bpm and that a beat is expected immediately
	bds->running_avg_wait 				= 0.5;
	bds->seconds_since_last_beat 		= 0.5;
	
	/* Clear remaining running averages */
	bds->frame_energy_running_avg 		= 0.0;
    for (int i = 0; i < BUFFER_SIZE / 2; i++)
		bds->running_avg_spectrum[i] = 0.0;
	
	return 0;
}

/* Take raw data from the mic and copy it into the designated 
 * array of double-precision floats, while also normalising 
 * the values to lie in the interval [-1, 1]. */
int copy_and_convert_buffer(int32_t *buffer, double *frame)
{
	assert(buffer != NULL);
	assert(frame  != NULL);
	
	for (int i = 0; i < BUFFER_SIZE; i++)
		frame[i] = (double)buffer[i] * NORM_FACTOR;
	
	return 0;
}

/* The beat detection task function. This function runs indefinitely.
 * First, initialises things, and then enters a while (true) loop,
 * whereupon it waits for data from the microphone, obtained in a
 * thread-safe manner using an RTOS queue. Then copies over and 
 * normalises this data, performs a fast Fourier transform on it
 * and hands it over to the beat detection function. If a beat is
 * detected, the GPIO pins are changed accordingly. */
void beat_detection_task(void *params)
{
	// Beat detection state struct. To be passed as pointer
	beat_detection_state bds;
	// Pointerto bufferof raw data recieved from mic
	sample_t *buffer;
	
	// Renormalised raw data
	double frame[BUFFER_SIZE];
	// Fourier transform of frame
	double spectrum[BUFFER_SIZE];
	
	// Initialising things
	kiss_fft_cfg kiss_fft_config = kiss_fft_alloc(BUFFER_SIZE, 0, 0, 0);
	
	init_hamming_window();
    init_angular_freq_weights();
    
	init_beat_detection_state(&bds);
	
	// The main loop
	while (true)
	{
		// Get a fresh buffer of samples from the mic
		xQueueReceive(data_queue, &buffer, portMAX_DELAY);
		
		// Copy over and renormalise the raw data
		copy_and_convert_buffer(buffer, frame);
		
		// Return the buffer array to the queue
		return_buffer(buffer);
		
		// Run the fast Fourier transform
		perform_fft(kiss_fft_config, frame, spectrum);
		
		// Self explanatory
		if (beat_detected(&bds, frame, spectrum))
			swap_leds();

	}
}

// Wrapper function for kiss fft
void perform_fft(kiss_fft_cfg kiss_fft_config, const double *frame, double *spectrum)
{
	assert(frame 	!= NULL);
	assert(spectrum != NULL);

	// Arrays for kiss_fft. Static to avoid stack overflow
	static kiss_fft_cpx fft_in [BUFFER_SIZE];
	static kiss_fft_cpx fft_out[BUFFER_SIZE];
	
	// Use the hamming window and the given frame to populate fft_in
	for (int i = 0; i < BUFFER_SIZE / 2; i++)
    {
        fft_in[i].r = frame[i + BUFFER_SIZE / 2] * hamming_window[i + BUFFER_SIZE / 2];
        fft_in[i].i = 0.0;
        fft_in[i + BUFFER_SIZE / 2].r = frame[i] * hamming_window[i];
        fft_in[i + BUFFER_SIZE / 2].i = 0.0;
    }
    
    // Calculate fft
	kiss_fft(kiss_fft_config, fft_in, fft_out);
	
	// Populate spectrum array with norms of complex numbers produced by kiss_fft
	for (int i = 0; i < BUFFER_SIZE; i++)
		spectrum[i] = sqrt(fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i);
}

/* 
 * Given the cosine of an angle, returns a number between 1 and 2 where
 * if the angle is close to 0 or pi, gives a number ~1, and, if the angle 
 * is close to pi/2 or -pi/2, gives a number ~m. The formula is (3 - cos(2x))/2
 * 
 */
double cos_angle_weighting_factor(double cos_angle, double m)
{
	/* Use the double-angle identity cos(2x) = 2(cos(x))^2 - 1 to avoid
	 * having to use acos and then cos to calculate cos(2x) (costly!!) */
	double cos_double_angle = 2.0 * sqr(cos_angle) - 1.0;
	
	return 1.0 + 0.5 * m - (0.5 * m - 1.0) * cos_double_angle;
}

/* Calculate the ``energy'' of the given frame using the raw data (not actually used - but still passed
 * so that it is accessible, should modifications be made), the fft thereof, and the state struct. */
double frame_energy(beat_detection_state *bds, const double *frame, const double *spectrum)
{
	/* The base energy number calculated from the amplitudes of 
	 * frequencies, with weights from amplitude_freq_weights */
    double amp_energy = 0;
    
    // Weighted L^2-norms of the current spectrum and the running average spectrum
    double weighted_spectrum_norm 			  = 0.0;
    double running_avg_spectrum_weighted_norm = 0.0;
    
    // Weighted L^2 inner product of the current spectrum and the running average spectrum
    double weighted_inner_product = 0.0;
    
    /* The (cosine of the) angle between the current spectrum and the running average
     * spectrum, considered as elements of the Hilbert space L^2(Z/256Z) where
     * Z/256Z is equipped with the measure given by the angular_freq_weights array */
	double cos_weighted_spectral_angle;
    
    /* Calculate the amplitude energy and weighted spectral norms */
    for (int i = 0; i < BUFFER_SIZE/2; i++)
    {
        amp_energy += amplitude_freq_weights[i] * spectrum[i];
        
        weighted_spectrum_norm 			   += angular_freq_weights[i] * sqr(spectrum[i]);
        running_avg_spectrum_weighted_norm += angular_freq_weights[i] * sqr(bds->running_avg_spectrum[i]);
	}
	
	// If the current spectrum is small in norm, set the angle to 0
	if (weighted_spectrum_norm < 0.1)
	{
		cos_weighted_spectral_angle = 1.0;
	}
	// If the running average spectrum is small in norm but the current isn't, set angle to pi/2
	else if (running_avg_spectrum_weighted_norm < 0.1)
	{
		cos_weighted_spectral_angle = 0.0;
	}
	/* Otherwise, proceed with the calculation, having avoided division
	 * by 0 and the numeric instability of dividing by small norms */
	else
	{
		// sqrt the norms
		weighted_spectrum_norm 			   = sqrt(weighted_spectrum_norm);
		running_avg_spectrum_weighted_norm = sqrt(running_avg_spectrum_weighted_norm);
		
		// Take the inner product
		for (int i = 0; i < BUFFER_SIZE/2; i++)
			weighted_inner_product += angular_freq_weights[i] * spectrum[i] * bds->running_avg_spectrum[i];
		
		// Calculate the resulting cosine
		cos_weighted_spectral_angle = weighted_inner_product / (weighted_spectrum_norm * running_avg_spectrum_weighted_norm);
	}

    return amp_energy * cos_angle_weighting_factor(cos_weighted_spectral_angle, ANGLE_WEIGHT);
}

/* 
 * Update the running beat period average.
 * 
 * This is done with care; detection of beats `between' beats, i.e., 
 * if an 8th note is picked up between a regular pulse of 4th notes,
 * this has an outsized effect on the running average beat period
 * if it's not weighted correctly, as it can happen, by definition,
 * more frequently than the regular pulse. Therefore simply taking
 * a convex combination of the running average with the current beat
 * period responds too strongly to spurious inter-pulse beats. Therefore
 * I take a convex combination with a different number, one modified
 * so as to be closer to the current running average that it would have
 * been otherwise in the case of suspiciously fast or slow beats.
 */

double normalised_current_wait(double running_avg, double t)
{
	return running_avg * (1 + WAIT_NORMALISATION_FACTOR * erf(2.0 * (t/running_avg - 1.0)));
}

void update_running_avg_wait(beat_detection_state *bds)
{
	assert(bds != NULL);
	
	bds->running_avg_wait = bds->running_avg_wait * RUNNING_AVG_WAIT_ADAPT_RATE
		+ normalised_current_wait(bds->seconds_since_last_beat, bds->running_avg_wait) * (1 - RUNNING_AVG_WAIT_ADAPT_RATE);
}

// Update therunning average spectrum. Nothing fancy here. Just a simple convex combination
void update_running_avg_spectrum(beat_detection_state *bds, const double *spectrum)
{
	assert(spectrum != NULL);
	
	for (int i = 0; i < BUFFER_SIZE / 2; i++)
		bds->running_avg_spectrum[i] = bds->running_avg_spectrum[i] *      RUNNING_AVG_SPECTRUM_ADAPT_RATE
													  + spectrum[i] * (1 - RUNNING_AVG_SPECTRUM_ADAPT_RATE);
}

// The main beat detection function. Returns 0 if no beat detected, and 1 if a beat is detected
int beat_detected(beat_detection_state *bds, const double *frame, const double *spectrum)
{
	assert(bds 		!= NULL);
	assert(frame 	!= NULL);
	assert(spectrum != NULL);
	
    int result;
    double energy, time_weighting;
    
    // Update the time since last beat
	bds->seconds_since_last_beat += BUFFER_DURATION_SEC;

	// Calculate the energy of the current frame
	energy = frame_energy(bds, frame, spectrum);
	
	// Calculate a weight that inhibits detecting beats unusually quickly, and disinhibits detecting beats unusually slowly
	time_weighting = pow(inv_erf_1 * erf(bds->seconds_since_last_beat / bds->running_avg_wait), TIME_WEIGHT_EXPONENT);
	
	// Cap it off at a max value; tinkering with TIME_WEIGHT_EXPONENT results in different limits as time -> inf, some much too high
	if (time_weighting > TIME_WEIGHT_MAX)
		time_weighting = TIME_WEIGHT_MAX;
	
	// Determine whether anything is happening and control whether the LEDs are on
	active = (bds->frame_energy_running_avg > ACTIVATION_THRESHOLD) || (energy > ACTIVATION_THRESHOLD);

	// Update the frame energy running average
	bds->frame_energy_running_avg = bds->frame_energy_running_avg * RUNNING_AVG_ENERGY_ADAPT_RATE + energy * (1 - RUNNING_AVG_ENERGY_ADAPT_RATE);
    
    
    /* If the energy, adjusted according to the time-based weighting, exceeds
     * beat_threshold times the running average energy, a beat is detected! */
    
    if (bds->frame_energy_running_avg > 0.1) // Avoid division by small numbers
		result = (energy * time_weighting) / bds->frame_energy_running_avg > bds->beat_threshold;
	else
		result = 0;
    
    if (result)
    {
		// If there is a beat in this frame, update some numbers
		update_running_avg_wait(bds);
		
		bds->seconds_since_last_beat = 0;
		bds->beat_threshold = bds->beat_threshold * BEAT_THRESHOLD_ADAPT_RATE + (1 - BEAT_THRESHOLD_ADAPT_RATE) * (energy / bds->frame_energy_running_avg);
	}
	else
	{
		// If no beat is detected, adjust the sensitivity
		bds->beat_threshold = ((bds->beat_threshold - 1) * BEAT_THRESHOLD_SENSITISE_RATE) + 1;
	}
	
	// Update the running average spectrum
	update_running_avg_spectrum(bds, spectrum);
	
	// Return the result
	return result;
}
