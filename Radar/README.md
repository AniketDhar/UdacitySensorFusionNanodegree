# FMCW Radar Target Detection using 2D FFT and CFAR

This project implements a radar signal processing pipeline using **FMCW (Frequency Modulated Continuous Wave)** techniques to detect a moving target's **range** and **velocity**. The final target detection is performed using a **Constant False Alarm Rate (CFAR)** filter applied on a **2D FFT Range-Doppler Map (RDM)**.

---

## 📌 Project Goals

- Simulate a radar environment with a moving target.
- Generate FMCW waveforms and simulate transmitted/received signals.
- Process the beat signal using **1D FFT** for range measurement.
- Process the beat signal using **2D FFT** for range-velocity (doppler) estimation.
- Apply **CFAR** to detect targets in noisy conditions.
- Visualize and interpret results at every stage.

---

## Pipeline Overview

The radar signal processing pipeline follows these key steps:

1. **Signal Simulation:**
   - Define radar and target parameters.
   - Generate transmitted (Tx) and received (Rx) signals based on time delays.
   - Generate the beat signal by mixing Tx and Rx.

2. **Range Measurement (1D FFT):**
   - Reshape the beat signal.
   - Perform FFT on each chirp (row) to estimate range.
   - Plot single-sided amplitude spectrum.

3. **Range-Doppler Map (2D FFT):**
   - Reshape the beat signal.
   - Perform 2D FFT across range and doppler dimensions.
   - Apply logarithmic scaling for better visualization.

4. **CFAR Detection:**
   - Slide a training-guard cell window over the RDM.
   - Estimate noise level, compute threshold.
   - Detect peaks above threshold.
   - Generate binary detection map.

---

## 🛠️ Implemented TODO Segments

### ✅ TODO 1: Signal Generation (Tx/Rx/Mix)
- Simulates the real-world mixing of transmitted and received radar signals.
- Beat frequency encodes range and velocity.

### ✅ TODO 2: Range Measurement (1D FFT)
- 1D FFT isolates the range information from beat signal.
- Only half of spectrum is kept (symmetry in FFT).

<img src="images/Range_from_first_FFT.png" width="700" height="400" />

### ✅ TODO 3: Range-Doppler Map (2D FFT)
- Produces 2D frequency spectrum for both range and Doppler.
- Provides insight into target's range and speed.

<img src="images/RDM_second_FFT.png" width="700" height="400" />

### ✅ TODO 4: CFAR Implementation
- Applies cell-averaging CFAR logic in a 2D sliding window.
- Ensures robust detection by adapting the threshold to local noise.

<img src="images/2DCFAR.png" width="700" height="400" />

## Parameters Used

|-|-|
|Parameter	        |Value          |
|Max Range	        |200 m          |
|Range Resolution	|1 m            |
|Max Velocity	    |100 m/s        |
|Carrier Freq	    |77 GHz         |
|# of Chirps	    |128            |
|# of Samples	    |1024           |
|Training Cells	    |Tr = 10, Tc = 8|
|Guard Cells	    |Gr = 4, Gc = 4 |
|CFAR Offset	    |6 dB           |


## Conclusion

This project simulates a full radar signal processing chain and successfully demonstrates how FMCW radar can estimate range and velocity of a target, even in noisy conditions. The CFAR filtering ensures that only high-confidence detections are retained, greatly reducing false alarms.