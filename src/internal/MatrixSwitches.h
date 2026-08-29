#ifndef __DCSBIOS_MATRIX_SWITCHES_H
#define __DCSBIOS_MATRIX_SWITCHES_H

#include "pico/stdlib.h"
#include "PollingInput.h"
#include "ExportStreamListener.h"
#include "SwitchMatrix.h"  // Must be adapted to RP2040 SDK

// Global matrix object
SwitchMatrix swPanel;

namespace DcsBios {

template <unsigned long pollIntervalMs = POLL_EVERY_TIME>
class Matrix2PosT : public PollingInput, public ResettableInput {
private:
	const char* msg_;
	uint row_;
	uint col_;
	char lastState_;
	bool reverse_;

	// This function initializes the member variables.
void init_(const char* msg, uint row, uint col, bool reverse) {
		msg_ = msg;
		row_ = row;
		col_ = col;
		lastState_ = swPanel.GetSwitchState(row_, col_);
		reverse_ = reverse;
	}

	// This function toggles the last state between 0 and -1.
void resetState() {
		lastState_ = (lastState_ == 0) ? -1 : 0;
	}

	// This function checks the switch state. If the reverse flag is set, it inverts the state. If the state changes, it attempts to send a message. If the attempt succeeds, it updates the last state.
void pollInput() {
		char state = swPanel.GetSwitchState(row_, col_);
		if (reverse_) state = !state;
		if (state != lastState_) {
			if (tryToSendDcsBiosMessage(msg_, state == false ? "0" : "1")) {
				lastState_ = state;
			}
		}
	}

public:
	Matrix2PosT(const char* msg, uint row, uint col, bool reverse) :
		PollingInput(pollIntervalMs) {
		init_(msg, row, col, reverse);
	}

	Matrix2PosT(const char* msg, uint row, uint col) :
		PollingInput(pollIntervalMs) {
		init_(msg, row, col, false);
	}

	// This function calls resetState.
void resetThisState() {
		resetState();
	}
};

typedef Matrix2PosT<> Matrix2Pos;

template <unsigned long pollIntervalMs = POLL_EVERY_TIME>
class Matrix3PosT : public PollingInput, public ResettableInput {
private:
	const char* msg_;
	uint rowA_, colA_;
	uint rowB_, colB_;
	char lastState_;

	char readState() {
		if (swPanel.GetSwitchState(rowA_, colA_) == true) return 0;
		if (swPanel.GetSwitchState(rowB_, colB_) == true) return 2;
		return 1;
	}

	void resetState() {
		lastState_ = (lastState_ == 0) ? -1 : 0;
	}

	void pollInput() {
		char state = readState();
		if (state != lastState_) {
			if (tryToSendDcsBiosMessage(msg_, (state == 0) ? "0" :
			                            (state == 1) ? "1" : "2")) {
				lastState_ = state;
			}
		}
	}

public:
	Matrix3PosT(const char* msg, uint rowA, uint colA, uint rowB, uint colB) :
		PollingInput(pollIntervalMs), msg_(msg), rowA_(rowA), colA_(colA), rowB_(rowB), colB_(colB) {
		lastState_ = readState();
	}

	void resetThisState() {
		resetState();
	}
};

typedef Matrix3PosT<> Matrix3Pos;

} // namespace DcsBios

#endif
