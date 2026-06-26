// Adapted from Uncertainty VU MeterBallistics class:
//	https://github.com/oamodular/uncertainty/blob/main/software/c%2B%2B/uncertainty_vu/uncertainty_vu.ino
//
// Changes:
//  - Converted to a template that receives a value type and sample rate
//	- Slewing based on a fall factor calculated from the sample rate and an input fall time
//
// Original license:
//   # Olivia Artz Modular Uncertainty License and Copyright Notices
//
//   Uncertainty is (c) 2023 by Olivia Artz Modular / OAM
//
//   This notice must be included in any distributions of this project or
//   derivative works.  Unless otherwise noted, this project is licensed under
//   Creative Commons Attribution-ShareAlike 4.0 International (CC BY-SA 4.0).
//   Full text available at https://creativecommons.org/licenses/by-sa/4.0/

#ifndef FOLLOWER_H_
#define FOLLOWER_H_

template<typename value_t, size_t SAMPLE_RATE>
class Follower {
protected:
	value_t _fallFactor;
public:
	value_t lastValue;

	Follower(float fallTimeInSeconds = 0.1f) {
		_fallFactor = value_t(1.0f / (fallTimeInSeconds * SAMPLE_RATE));
		lastValue = value_t(0);
	}

	value_t Process(value_t in) {
		// get magnitude of input
		in = in > value_t(0) ? in : -in;
		// how much did the magnitude of the input value change since last time?
		value_t delta = in - lastValue;
		// if we're decreasing, seriously limit how quickly we can decrease
		if (delta < value_t(0)) {
			lastValue += delta * _fallFactor;
		} else { // otherwise just adjust by the difference, mimicking the input
			lastValue += delta;
		}
		return lastValue;
	}
};

#endif //FOLLOWER_H_

