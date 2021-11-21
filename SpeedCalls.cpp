// Downloaded from https://developer.x-plane.com/code-sample/openal-example/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctime> 

#include "XPLMMenus.h"
#include "XPLMUtilities.h"
#include "XPLMProcessing.h"
#include "XPLMDataAccess.h"
#include "XPLMPlugin.h"

// OS X: we use this to convert our file path.
#if APL
#include <Carbon/Carbon.h>
#endif

// Your include paths for OpenAL may vary by platform.
#include <al.h>
#include <alc.h>

/**************************************************************************************************************
 * WAVE FILE LOADING
 **************************************************************************************************************/

 // You can just use alutCreateBufferFromFile to load a wave file, but there seems to be a lot of problems with 
 // alut not beign available, being deprecated, etc.  So...here's a stupid routine to load a wave file.  I have
 // tested this only on x86 machines, so if you find a bug on PPC please let me know.

 // Macros to swap endian-values.

#define SWAP_32(value)                 \
        (((((unsigned short)value)<<8) & 0xFF00)   | \
         ((((unsigned short)value)>>8) & 0x00FF))

#define SWAP_16(value)                     \
        (((((unsigned int)value)<<24) & 0xFF000000)  | \
         ((((unsigned int)value)<< 8) & 0x00FF0000)  | \
         ((((unsigned int)value)>> 8) & 0x0000FF00)  | \
         ((((unsigned int)value)>>24) & 0x000000FF))


/* File to write data to. */
static FILE *	gOutputFile = NULL;
static char		outputPath[255];

/* Identity of the aircraft. */
static char AircraftIdentity[40];

/* Time for log */
static time_t current_time;
static struct tm * ti;

/* Data refs we will record. */
static XPLMDataRef	gDataRefIAS = NULL;
static XPLMDataRef	gDataRefFlaps = NULL;
static XPLMDataRef	gDataRefRadioAltitude = NULL;
static XPLMDataRef	gDataRefGearRatio = NULL;
static XPLMDataRef	gDataRefVerticalSpeed = NULL;
static XPLMDataRef	gDataRefICAO = NULL;
static XPLMDataRef	gDataRefTakeOffWeight = NULL;
static XPLMDataRef	gDataRefRunningTime = NULL;

/* speed at a t time */
static int lastMemorizedSpeed = NULL;

/* radio height at a t time */
static int lastRadioAltitude = NULL;

/* gear ratio at a t time */
static float lastGearRatio = NULL;

/* Speed */
static int V1 = 0;
static int VR = 0;
static int V2 = 0;


// functions
static float	MyFlightLoopCallback(
	float                inElapsedSinceLastCall,
	float                inElapsedTimeSinceLastFlightLoop,
	int                  inCounter,
	void *               inRefcon);

static void playSound();

static void aircraftIdentity();     // To stamp the identity of the ariplane in a file

static void speedCalculation();     // V1, VR .... calculation


// Wave files are RIFF files, which are "chunky" - each section has an ID and a length.  This lets us skip
// things we can't understand to find the parts we want.  This header is common to all RIFF chunks.
struct chunk_header {
	int			id;
	int			size;
};

// WAVE file format info.  We pass this through to OpenAL so we can support mono/stereo, 8/16/bit, etc.
struct format_info {
	short		format;				// PCM = 1, not sure what other values are legal.
	short		num_channels;
	int			sample_rate;
	int			byte_rate;
	short		block_align;
	short		bits_per_sample;
};

// This utility returns the start of data for a chunk given a range of bytes it might be within.  Pass 1 for
// swapped if the machine is not the same endian as the file.
static char *	find_chunk(char * file_begin, char * file_end, int desired_id, int swapped)
{
	while (file_begin < file_end)
	{
		chunk_header * h = (chunk_header *)file_begin;
		if (h->id == desired_id && !swapped)
			return file_begin + sizeof(chunk_header);
		if (h->id == SWAP_32(desired_id) && swapped)
			return file_begin + sizeof(chunk_header);
		int chunk_size = swapped ? SWAP_32(h->size) : h->size;
		char * next = file_begin + chunk_size + sizeof(chunk_header);
		if (next > file_end || next <= file_begin)
			return NULL;
		file_begin = next;
	}
	return NULL;
}

// Given a chunk, find its end by going back to the header.
static char * chunk_end(char * chunk_start, int swapped)
{
	chunk_header * h = (chunk_header *)(chunk_start - sizeof(chunk_header));
	return chunk_start + (swapped ? SWAP_32(h->size) : h->size);
}

#define FAIL(X) { XPLMDebugString(X); free(mem); return 0; }

#define RIFF_ID 0x46464952			// 'RIFF'
#define FMT_ID  0x20746D66			// 'FMT '
#define DATA_ID 0x61746164			// 'DATA'

ALuint load_wave(const char * file_name)
{
	// First: we open the file and copy it into a single large memory buffer for processing.

	FILE * fi = fopen(file_name, "rb");
	if (fi == NULL)
	{
		XPLMDebugString("WAVE file load failed - could not open.\n");
		return 0;
	}
	fseek(fi, 0, SEEK_END);
	int file_size = ftell(fi);
	fseek(fi, 0, SEEK_SET);
	char * mem = (char*)malloc(file_size);
	if (mem == NULL)
	{
		XPLMDebugString("WAVE file load failed - could not allocate memory.\n");
		fclose(fi);
		return 0;
	}
	if (fread(mem, 1, file_size, fi) != file_size)
	{
		XPLMDebugString("WAVE file load failed - could not read file.\n");
		free(mem);
		fclose(fi);
		return 0;
	}
	fclose(fi);
	char * mem_end = mem + file_size;

	// Second: find the RIFF chunk.  Note that by searching for RIFF both normal
	// and reversed, we can automatically determine the endian swap situation for
	// this file regardless of what machine we are on.

	int swapped = 0;
	char * riff = find_chunk(mem, mem_end, RIFF_ID, 0);
	if (riff == NULL)
	{
		riff = find_chunk(mem, mem_end, RIFF_ID, 1);
		if (riff)
			swapped = 1;
		else
			FAIL("Could not find RIFF chunk in wave file.\n")
	}

	// The wave chunk isn't really a chunk at all. :-(  It's just a "WAVE" tag 
	// followed by more chunks.  This strikes me as totally inconsistent, but
	// anyway, confirm the WAVE ID and move on.

	if (riff[0] != 'W' ||
		riff[1] != 'A' ||
		riff[2] != 'V' ||
		riff[3] != 'E')
		FAIL("Could not find WAVE signature in wave file.\n")

		char * format = find_chunk(riff + 4, chunk_end(riff, swapped), FMT_ID, swapped);
	if (format == NULL)
		FAIL("Could not find FMT  chunk in wave file.\n")

		// Find the format chunk, and swap the values if needed.  This gives us our real format.

		format_info * fmt = (format_info *)format;
	if (swapped)
	{
		fmt->format = SWAP_16(fmt->format);
		fmt->num_channels = SWAP_16(fmt->num_channels);
		fmt->sample_rate = SWAP_32(fmt->sample_rate);
		fmt->byte_rate = SWAP_32(fmt->byte_rate);
		fmt->block_align = SWAP_16(fmt->block_align);
		fmt->bits_per_sample = SWAP_16(fmt->bits_per_sample);
	}

	// Reject things we don't understand...expand this code to support weirder audio formats.

	if (fmt->format != 1) FAIL("Wave file is not PCM format data.\n")
		if (fmt->num_channels != 1 && fmt->num_channels != 2) FAIL("Must have mono or stereo sound.\n")
			if (fmt->bits_per_sample != 8 && fmt->bits_per_sample != 16) FAIL("Must have 8 or 16 bit sounds.\n")
				char * data = find_chunk(riff + 4, chunk_end(riff, swapped), DATA_ID, swapped);
	if (data == NULL)
		FAIL("I could not find the DATA chunk.\n")

		int sample_size = fmt->num_channels * fmt->bits_per_sample / 8;
	int data_bytes = chunk_end(data, swapped) - data;
	int data_samples = data_bytes / sample_size;

	// If the file is swapped and we have 16-bit audio, we need to endian-swap the audio too or we'll 
	// get something that sounds just astoundingly bad!

	if (fmt->bits_per_sample == 16 && swapped)
	{
		short * ptr = (short *)data;
		int words = data_samples * fmt->num_channels;
		while (words--)
		{
			*ptr = SWAP_16(*ptr);
			++ptr;
		}
	}

	// Finally, the OpenAL crud.  Build a new OpenAL buffer and send the data to OpenAL, passing in
	// OpenAL format enums based on the format chunk.

	ALuint buf_id = 0;
	alGenBuffers(1, &buf_id);
	if (buf_id == 0) FAIL("Could not generate buffer id.\n");

	alBufferData(buf_id, fmt->bits_per_sample == 16 ?
		(fmt->num_channels == 2 ? AL_FORMAT_STEREO16 : AL_FORMAT_MONO16) :
		(fmt->num_channels == 2 ? AL_FORMAT_STEREO8 : AL_FORMAT_MONO8),
		data, data_bytes, fmt->sample_rate);
	free(mem);
	return buf_id;
}




/**************************************************************************************************************
 * SAMPLE OEPNAL PLUGIN:
 **************************************************************************************************************/

static ALuint			snd_src = 0;			// Sample source and buffer - this is one "sound" we play.
static ALuint			snd_buffer = 0;
static float			pitch = 1.0f;			// Start with 1.0 pitch - no pitch shift.

static ALCdevice *		my_dev = NULL;			// We make our own device and context to play sound through.
static ALCcontext *		my_ctx = NULL;

// This is a stupid logging error function...useful for debugging, but not good error checking.
static void CHECK_ERR(void)
{
	ALuint e = alGetError();
	if (e != AL_NO_ERROR)
		printf("ERROR: %d\n", e);
}

// Mac specific: this converts file paths from HFS (which we get from the SDK) to Unix (which the OS wants).
// See this for more info:
//
// http://www.xsquawkbox.net/xpsdk/mediawiki/FilePathsAndMacho

#if APL
int ConvertPath(const char * inPath, char * outPath, int outPathMaxLen) {

	CFStringRef inStr = CFStringCreateWithCString(kCFAllocatorDefault, inPath, kCFStringEncodingMacRoman);
	if (inStr == NULL)
		return -1;
	CFURLRef url = CFURLCreateWithFileSystemPath(kCFAllocatorDefault, inStr, kCFURLHFSPathStyle, 0);
	CFStringRef outStr = CFURLCopyFileSystemPath(url, kCFURLPOSIXPathStyle);
	if (!CFStringGetCString(outStr, outPath, outPathMaxLen, kCFURLPOSIXPathStyle))
		return -1;
	CFRelease(outStr);
	CFRelease(url);
	CFRelease(inStr);
	return 0;
}
#endif

// Initialization code.

static float init_sound(char * soundFile)
{
	char buf[2048];

	// We have to save the old context and restore it later, so that we don't interfere with X-Plane
	// and other plugins.

	ALCcontext * old_ctx = alcGetCurrentContext();

	// Try to create our own default device and context.  If we fail, we're dead, we won't play any sound.

	my_dev = alcOpenDevice(NULL);
	if (my_dev == NULL)
	{
		XPLMDebugString("Could not open the default OpenAL device.\n");
		return 0;
	}
	my_ctx = alcCreateContext(my_dev, NULL);
	if (my_ctx == NULL)
	{
		if (old_ctx)
			alcMakeContextCurrent(old_ctx);
		alcCloseDevice(my_dev);
		my_dev = NULL;
		XPLMDebugString("Could not create a context.\n");
		return 0;
	}

	// Make our context current, so that OpenAL commands affect our, um, stuff.

	alcMakeContextCurrent(my_ctx);

	ALfloat	zero[3] = { 0 };

	// Build a path to "wave" in our parent directory.	
	char dirchar = *XPLMGetDirectorySeparator();
	XPLMGetPluginInfo(XPLMGetMyID(), NULL, buf, NULL, NULL);
	char * p = buf;
	char * slash = p;
	while (*p)
	{
		if (*p == dirchar) slash = p;
		++p;
	}
	++slash;
	*slash = 0;
	strcat(buf, soundFile);
#if APL
	ConvertPath(buf, buf, sizeof(buf));
#endif

	// Generate 1 source and load a buffer of audio.
	alGenSources(1, &snd_src);
	CHECK_ERR();
	snd_buffer = load_wave(buf);
	CHECK_ERR();

	// Basic initializtion code to play a sound: specify the buffer the source is playing, as well as some 
	// sound parameters. This doesn't play the sound - it's just one-time initialization.
	alSourcei(snd_src, AL_BUFFER, snd_buffer);
	alSourcef(snd_src, AL_PITCH, 1.0f);
	alSourcef(snd_src, AL_GAIN, 0.3f);
	alSourcei(snd_src, AL_LOOPING, 0);
	alSourcefv(snd_src, AL_POSITION, zero);
	alSourcefv(snd_src, AL_VELOCITY, zero);
	CHECK_ERR();

	// Finally: put back the old context _if_ we had one.  If old_ctx was null, X-Plane isn't using OpenAL.

	if (old_ctx)
		alcMakeContextCurrent(old_ctx);
	return 0.0f;
}

PLUGIN_API int XPluginStart(char * name, char * sig, char * desc)
{

	XPLMGetSystemPath(outputPath);
	strcat(outputPath, "AirplaneIdentity.txt");
	gOutputFile = fopen(outputPath, "w");


	strcpy(name, "SpeedCalls");
	strcpy(sig, "xpsdk.demo.openal.speedcalls");
	strcpy(desc, "Speed Calls 80, 100, V1, VR, V2 and Gear");

	if (sizeof(unsigned int) != 4 ||
		sizeof(unsigned short) != 2)
	{
		XPLMDebugString("This plugin was compiled with a compiler with weird type sizes.\n");
		return 0;
	}

	/* Find the data refs we want to record. */
	gDataRefICAO = XPLMFindDataRef("sim/aircraft/view/acf_ICAO");
	gDataRefIAS = XPLMFindDataRef("sim/flightmodel/position/indicated_airspeed");
	gDataRefFlaps = XPLMFindDataRef("sim/flightmodel/controls/flaprqst");
	gDataRefGearRatio = XPLMFindDataRef("sim/flightmodel/movingparts/gear1def");
	gDataRefVerticalSpeed = XPLMFindDataRef("sim/cockpit2/gauges/indicators/vvi_fpm_pilot");
	gDataRefTakeOffWeight = XPLMFindDataRef("sim/flightmodel/weight/m_total");
	gDataRefRunningTime = XPLMFindDataRef("sim/time/total_running_time_sec");
	gDataRefRadioAltitude = XPLMFindDataRef("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot");

	lastMemorizedSpeed = 0;
	lastRadioAltitude = 0;
	lastGearRatio = 1.0;        // descending order

	XPLMRegisterFlightLoopCallback(
		MyFlightLoopCallback,	/* Callback */
		1.0,					/* Interval */
		NULL);					/* refcon not used. */


	return 1;
}

PLUGIN_API void XPluginStop(void)
{
	// Cleanup: nuke our context if we have it.  This is hacky and bad - we should really destroy
	// our buffers and sources.  I have _no_ idea if OpenAL will leak memory.
	if (my_ctx) alcDestroyContext(my_ctx);
	if (my_dev) alcCloseDevice(my_dev);
	my_ctx = NULL;
	my_dev = NULL;

	/* Close the file if it's not already done */
	fclose(gOutputFile);
}

PLUGIN_API int XPluginEnable(void)
{
	return 1;
}

PLUGIN_API void XPluginDisable(void)
{

}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID from, int msg, void * p)
{

}

float	MyFlightLoopCallback(
	float                inElapsedSinceLastCall,
	float                inElapsedTimeSinceLastFlightLoop,
	int                  inCounter,
	void *               inRefcon)
{
	/* The actual callback.  First we read the data. */

	int runningTime = (int)XPLMGetDataf(gDataRefRunningTime);

	int IAS = (int)XPLMGetDataf(gDataRefIAS);
	int radioAltitude = (int)XPLMGetDataf(gDataRefRadioAltitude);
	int verticalSpeed = (int)XPLMGetDataf(gDataRefVerticalSpeed);
	float levelFlaps = XPLMGetDataf(gDataRefFlaps);
	float gearRatio = XPLMGetDataf(gDataRefGearRatio);

	if (runningTime > 0 && runningTime < 2) {
		aircraftIdentity();
	}

	if (IAS == 0) {				
		speedCalculation();
	}

	// at takeoff
	if (radioAltitude < 10 && levelFlaps < .7f) {		// on the ground and flaps <= 20 degrees

		if (IAS >= 75 && lastMemorizedSpeed < 75) {		// check with the last memorized speed
			init_sound("v80.wav");
			lastMemorizedSpeed = IAS;
			playSound();
		}

		if (IAS >= 95 && lastMemorizedSpeed < 95) {
			init_sound("v100.wav");
			lastMemorizedSpeed = IAS;
			playSound();
		}

		if (V1 != -999) {   // no calls of no data

			if (IAS >= V1 && VR - V1 < 4 && lastMemorizedSpeed < V1) {  // concatenation of the calls because it's to close
				init_sound("v1vr.wav");
				lastMemorizedSpeed = IAS;
				playSound();
			}

			if (IAS >= V1 && lastMemorizedSpeed < V1) {
				init_sound("v1.wav");
				lastMemorizedSpeed = IAS;
				playSound();
			}

			if (IAS >= VR && lastMemorizedSpeed < VR) {

				init_sound("vr.wav");
				lastMemorizedSpeed = IAS;
				playSound();
			}

			if (IAS >= V2 && lastMemorizedSpeed < V2) {
				init_sound("v2.wav");
				lastMemorizedSpeed = IAS;
				playSound();
			}
		}
	}


	// on climbing
	if (radioAltitude < 1500 && radioAltitude >= 50 && verticalSpeed > 100) {

		if (gearRatio == 1.0f && lastGearRatio == 1.0f) {
			init_sound("GearUpCall.wav");
			lastGearRatio = lastGearRatio - .1f;
			playSound();
		}

		if (gearRatio < .5f && lastGearRatio > .5f) {
			init_sound("GearGoingUp.wav");
			lastGearRatio = gearRatio;
			playSound();
		}

		if (gearRatio < .01f && lastGearRatio > .01f) {
			init_sound("GearUpLocked.wav");
			lastGearRatio = gearRatio;
			playSound();
		}
	}


	// on landing
	if (radioAltitude < 10 && levelFlaps > .7f) {      // on final and flaps > 20 degrees

		if (IAS <= 105 && lastMemorizedSpeed > 105) {
			init_sound("v100.wav");
			lastMemorizedSpeed = IAS;
			playSound();
		}

		if (IAS <= 85 && lastMemorizedSpeed > 85) {   // check with the last memorized speed
			init_sound("v80.wav");
			lastMemorizedSpeed = IAS;
			playSound();
		}

	}
	/* Return 1.0 to indicate that we want to be called again in 1 second. */
	return 1.0;
}


void playSound() {

	if (my_ctx)
	{
		// An example of actually playing the sound.  First we change to our 
		// context, then we play the sound at pitch, then we change the context back.
		// We check for null contexts both for us (our init failed) and the old context
		// (X-plane's sound failed).

		ALCcontext * old_ctx = alcGetCurrentContext();
		alcMakeContextCurrent(my_ctx);

		alSourcef(snd_src, AL_PITCH, pitch);
		alSourcePlay(snd_src);

		CHECK_ERR();

		if (old_ctx)
			alcMakeContextCurrent(old_ctx);
	}
}

void aircraftIdentity() {

	// http://www.xsquawkbox.net/xpsdk/mediawiki/IdentifyingYourAircraft
	
	// Stamp aircraftIdentity and time in file
	int ByteVals[40];
	XPLMGetDatab(gDataRefICAO, ByteVals, 0, 40);
	strcpy(AircraftIdentity, (char*)&ByteVals);
	fprintf(gOutputFile, "%s\n", AircraftIdentity);

	time(&current_time);
	ti = localtime(&current_time);
	fprintf(gOutputFile, "\n%s\n", asctime(ti));

	fflush(gOutputFile);
	fclose(gOutputFile);
}

void speedCalculation() {

		// All calculations are made by regression equation
		// Linear regression attempts to model the relationship between 
		// two variables by fitting a linear equation to observed data.
		// A linear regression line has an equation of the form y = a + bx
		// b = [n(∑xiyi) − (∑xi)(∑yi) ] /  [ n(∑xi^2) − (∑xi)^2 ] and a = y − bx

		// There is a definite relationship between takeOffWeight and Speed when you look at the tables 
		// and it's confirmed by the regression coefficients, as you can see below.

		// https://calculis.net/droite-regression-lineaire

	if (strstr(AircraftIdentity, "B744") != NULL) {

		// from the table : http://b747400.com/pdf/told.pdf

		int takeOffWeight = ((int)XPLMGetDataf(gDataRefTakeOffWeight)) / 1000;  // tables in tons
		float levelFlaps = XPLMGetDataf(gDataRefFlaps);

		if (levelFlaps < .55f)	// flaps 10
		{
			// Le coefficient de corrélation est r = 0.9981612624668

			V1 = (int)(0.29772961816305 * takeOffWeight + 40.048503611971);
			VR = (int)(0.33519091847265 * takeOffWeight + 44.303749570003);
			V2 = (int)(0.26212590299278 * takeOffWeight + 85.208118335053);
		}
		else					// flaps 20
		{
			V1 = (int)(0.29587203302374 * takeOffWeight + 35.633642930857);
			VR = (int)(0.32765737874097 * takeOffWeight + 40.84348125215);
			V2 = (int)(0.25789473684211 * takeOffWeight + 80.152046783626);
		}
	}
	else if ((strstr(AircraftIdentity, "B788") != NULL))
	{
		// from the table : https://forums.x-plane.org/index.php?/files/file/43387-b787-9-takoff-and-landing-data-cards-for-the-magknight-b789/

		int takeOffWeight = ((int)XPLMGetDataf(gDataRefTakeOffWeight) * 2.20462f) / 1000;  // table in LBS
		float levelFlaps = XPLMGetDataf(gDataRefFlaps);

		if (levelFlaps > .3f  && levelFlaps < .4f)				// flaps 5
		{
			// Le coefficient de corrélation est r = 0.9981612624668

			V1 = (int)(0.21190476190476 * takeOffWeight + 54.238095238095);   // tables are in LBS
			VR = (int)(0.20982142857143 * takeOffWeight + 58.696428571429);
			V2 = (int)(0.17261904761905 * takeOffWeight + 82.702380952381);
		}
		else if (levelFlaps > .45f && levelFlaps < .6f)					// flaps 15
		{
			V1 = (int)(0.20625 * takeOffWeight + 51.375);
			VR = (int)(0.20089285714286 * takeOffWeight + 56.767857142857);
			V2 = (int)(0.16875 * takeOffWeight + 78.625);
		}
		else															// flaps 20 
		{
			V1 = (int)(0.19553571428571 * takeOffWeight + 50.160714285714);
			VR = (int)(0.19494047619048 * takeOffWeight + 52.64880952381);
			V2 = (int)(0.15982142857143 * takeOffWeight + 76.196428571429);
		}
	}

	else if ((strstr(AircraftIdentity, "B752") != NULL))
	{
		// from the table : https://community.infiniteflight.com/t/boeing-b757-200-performance-charts/534873

		int takeOffWeight = ((int)XPLMGetDataf(gDataRefTakeOffWeight) * 2.20462f) / 1000;  // table in LBS
		float levelFlaps = XPLMGetDataf(gDataRefFlaps);

		if (levelFlaps > .3f  && levelFlaps < .4f)				// flaps 5
		{
			// Le coefficient de corrélation est r = 0.99910392134243

			V1 = (int)(0.45833333333333 * takeOffWeight + 48.25);   // tables are in LBS
			VR = (int)(0.45833333333333 * takeOffWeight + 51.25);
			V2 = (int)(0.36309523809524 * takeOffWeight + 75.071428571429);
		}
		else if (levelFlaps > .45f && levelFlaps < .6f)					// flaps 15
		{
			V1 = (int)(0.42857142857143 * takeOffWeight + 46.857142857143);
			VR = (int)(0.41785714285714 * takeOffWeight + 52.142857142857);
			V2 = (int)(0.36071428571429 * takeOffWeight + 68);
		}
		else															// flaps 20 
		{
			V1 = (int)(0.4 * takeOffWeight + 45);
			VR = (int)(0.4 * takeOffWeight + 48);
			V2 = (int)(0.33928571428571 * takeOffWeight + 65.285714285714);
		}
	}

	else if ((strstr(AircraftIdentity, "B738") != NULL))
	{
		// from the table :  https://fr.scribd.com/document/356588839/Delta-Virtual-Airline-B737-Manual-pdf

		// Le coefficient de corrélation est r = 0.99926117463131
		int takeOffWeight = ((int)XPLMGetDataf(gDataRefTakeOffWeight) * 2.20462f) / 1000;  // table in LBS
		float levelFlaps = XPLMGetDataf(gDataRefFlaps);

		V1 = (int)(0.52 * takeOffWeight + 59.8);   // flaps 10
		VR = (int)(0.53 * takeOffWeight + 60.5);
		V2 = (int)(0.4 * takeOffWeight + 89);
	}

	else if ((strstr(AircraftIdentity, "CL30") != NULL))
	{

		// from the table :  http://www.dream-air.ru/new/pilotam/CL300AFM_Rev10_TR49.pdf

		// Le coefficient de corrélation est r = 0.99773567023206

		int takeOffWeight = ((int)XPLMGetDataf(gDataRefTakeOffWeight) * 2.20462f) / 1000;  // table in LBS
		float levelFlaps = XPLMGetDataf(gDataRefFlaps);

		V1 = (int)(2.4714285714286 * takeOffWeight + 37.942857142857);   // flaps 10
		VR = (int)(2.2857142857143 * takeOffWeight + 46.238095238095);
		V2 = (int)(1.5571428571429 * takeOffWeight + 78.780952380952);

	}
	else if ((strstr(AircraftIdentity, "MD82") != NULL))
	{

		// from the table :  http://www.dream-air.ru/new/pilotam/CL300AFM_Rev10_TR49.pdf

		// Le coefficient de corrélation est r = 0.99773567023206

		int takeOffWeight = ((int)XPLMGetDataf(gDataRefTakeOffWeight) * 2.20462f) / 1000;  // table in LBS
		float levelFlaps = XPLMGetDataf(gDataRefFlaps);

		V1 = (int)(2.4714285714286 * takeOffWeight + 37.942857142857);   // flaps 10
		VR = (int)(2.2857142857143 * takeOffWeight + 46.238095238095);
		V2 = (int)(1.5571428571429 * takeOffWeight + 78.780952380952);

	}

	else {

		V1 = -999;    // no data

	}

}



