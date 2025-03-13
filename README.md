# AAE6102
AAE6102 asg1
# AAE6102 Assigment1 Report from CHENG LI

## Task 1: Acquisition
The goal of acquisition is to identify observable satellites and approximate values **carrier phase** and **code phase** of the satellites signals.  

**code phase**: To create a local PRN code that synchronizes with the incoming code.  

**carrier phase**: For downconversion (including doppler effect) 

Acquisition algorithm : **Parallel Code Phase search acquisition**  

**Step 1**: Generate carrier frequency with different doppler shift, the freqstep is 500 Hz here, the frequency range is -7k HZ to 7K hz.
```
  frqBins(frqBinIndex) = settings.IF - settings.acqSearchBand + ...
                                             0.5e3 * (frqBinIndex - 1);
```
**Step 2**：Parallel Code Phase search acquisition for one satellite with 20 ms data
```
  caCodeFreqDom = conj(fft(caCodesTable(PRN, :)));
......
  IQfreqDom1 = fft(I1 + 1i*Q1);
  IQfreqDom2 = fft(I2 + 1i*Q2);
......
 convCodeIQ1 = IQfreqDom1 .* caCodeFreqDom;
 convCodeIQ2 = IQfreqDom2 .* caCodeFreqDom;
......
   acqRes1 = abs(ifft(convCodeIQ1));
   acqRes2 = abs(ifft(convCodeIQ2));
```
**Step 3**：Setting the acquisition threshold and go to fine acquisition
### Acquisiton result from data open sky

**7 GPS Satellites are acquired with code delay and doppler shown below**
![image](https://github.com/EinsteinC/AAE6102/blob/main/13.JPG)


![image](https://github.com/EinsteinC/AAE6102/blob/main/12.JPG)

### Acquistion result from urban

** Four Satellites are acquired**

![image](https://github.com/EinsteinC/AAE6102/blob/main/2.JPG)
![image](https://github.com/EinsteinC/AAE6102/blob/main/11.JPG)

Comapred to open-sky environments, less GPS satellites are acquired due to the blockage of buildings.

## Task 2: Tracking
### 2.1 Code Analysis
Tracking aims to fine-tune carrier frequency and code phase values for consistent monitoring.

In code tracking with early, late, and prompt discriminators spaced at 0.5 chips, the correlator output can be calculated by comparing the received signal with the locally generated code at different points in time.

The correlator output for the prompt, early, and late correlations can be calculated using the following formula:

Correlation Output = Σ [Received Signal * Locally Generated Code]

The prompt correlation is calculated by aligning the locally generated code with the received signal at the current time. The early correlation is calculated by aligning the locally generated code slightly earlier, and the late correlation is calculated by aligning the locally generated code slightly later.

By computing these correlations, the tracking system can estimate the alignment between the received signal and the local code, allowing adjustments to be made to refine the carrier frequency and code phase values for accurate tracking.
```
            I_E = sum(earlyCode  .* iBasebandSignal);
            Q_E = sum(earlyCode  .* qBasebandSignal);
            I_P = sum(promptCode .* iBasebandSignal);
            Q_P = sum(promptCode .* qBasebandSignal);
            I_L = sum(lateCode   .* iBasebandSignal);
            Q_L = sum(lateCode   .* qBasebandSignal);

```
Using Delay Lock Loop discriminator to adjust the code phase
```
 codeError = (sqrt(I_E * I_E + Q_E * Q_E) - sqrt(I_L * I_L + Q_L * Q_L)) / ...
                (sqrt(I_E * I_E + Q_E * Q_E) + sqrt(I_L * I_L + Q_L * Q_L));
```
### 2.2 Multi-correlator Gerneration

**To obtain the Autocorrelation function, multiple correlators must be implemented**
Here, multiple correlator with spacing 0.1 chip from -0.5 chip to 0.5 chip are applied
```
            tcode       = (remCodePhase-0.4) : codePhaseStep : ((blksize-1)*codePhaseStep+remCodePhase-0.4);
            tcode2      = ceil(tcode) + 1;
            earlyCode04    = caCode(tcode2);
            tcode       = (remCodePhase-0.3) : codePhaseStep : ((blksize-1)*codePhaseStep+remCodePhase-0.3);
            tcode2      = ceil(tcode) + 1;
            earlyCode03    = caCode(tcode2);
            tcode       = (remCodePhase-0.2) : codePhaseStep : ((blksize-1)*codePhaseStep+remCodePhase-0.2);
            tcode2      = ceil(tcode) + 1;
            earlyCode02    = caCode(tcode2);
            tcode       = (remCodePhase-0.1) : codePhaseStep : ((blksize-1)*codePhaseStep+remCodePhase-0.1);
            tcode2      = ceil(tcode) + 1;
            earlyCode01    = caCode(tcode2);
            tcode       = (remCodePhase+0.1) : codePhaseStep : ((blksize-1)*codePhaseStep+remCodePhase+0.1);
            tcode2      = ceil(tcode) + 1;
            lateCode01    = caCode(tcode2);
            tcode       = (remCodePhase+0.2) : codePhaseStep : ((blksize-1)*codePhaseStep+remCodePhase+0.2);
            tcode2      = ceil(tcode) + 1;
            lateCode02   = caCode(tcode2);
            tcode       = (remCodePhase+0.3) : codePhaseStep : ((blksize-1)*codePhaseStep+remCodePhase+0.3);
            tcode2      = ceil(tcode) + 1;
            lateCode03    = caCode(tcode2);
            tcode       = (remCodePhase+0.4) : codePhaseStep : ((blksize-1)*codePhaseStep+remCodePhase+0.4);
            tcode2      = ceil(tcode) + 1;
            lateCode04   = caCode(tcode2);

```

### 2.3 Tracking result from data open sky (PRN 16 as an Example)
![image](https://github.com/EinsteinC/AAE6102/blob/main/5.JPG)

The Q-channel approaching zero signifies that it primarily comprises noise and residual error, fluctuating around zero, validating the alignment of the carrier phase.

When the DLL output (Code Discriminator Output) is close to zero, it signifies that the local code aligns with the received signal's code phase, indicating minimal code tracking error.

An PLL output (Phase/Frequency Discriminator Output) near zero indicates that the local carrier is in sync with the received signal carrier, ensuring stable carrier tracking.

A significantly higher Prompt correlation compared to Early/Late correlations indicates precise alignment of code phases, affirming the DLL's stable tracking state.

**Autocorrelation Function from Multi-correlator output**

The shape of ACF is symmetric and undistorted, indicating the satellite is undistorted and not under the influence of multipath, which is in agreement with the experiment scenario (Open Sky);
**Above proves that the satellite in open sky is well acquired and tracked**

### 2,4 Tracking result from data urban 
![image](https://github.com/EinsteinC/AAE6102/blob/main/7.JPG)

The Q-channel deviating significantly from zero, occasionally exceeding the I-channel, suggests that energy is not solely concentrated on the I-channel, indicating that the carrier phase may not always be well-aligned.

If the DLL output (Code Discriminator Output) remains consistent with that in an open-sky scenario, it implies that the local code alignment with the received signal code phase is maintained.

Variations in the PLL output, occasionally deviating significantly from zero and showing notable increases at times, indicate that the local carrier may not always be synchronized with the received signal carrier, resulting in unstable carrier tracking.

In instances where the Prompt correlation is not consistently significantly greater than Early/Late correlations, and even weaker at times (especially when PLL values are higher), it suggests that carrier phases are not precisely aligned.

As the multi-correlator output is asymmetric, indicating that multipath effects distort the shape of the Auto-Correlation Function (ACF), incorrect pseudorange measurements may result, leading to reduced positioning accuracy.

**Above proves that the satellite in urban is not well acquired and tracked**

## Task 3: Navigation data decoding (PRN 16 Open-Sky and PRN18 urban as an Example)
![image](https://github.com/EinsteinC/AAE6102/blob/main/5.JPG)

Above is the navigation data message decoded from incoming signal of open sky.

![image](https://github.com/EinsteinC/AAE6102/blob/main/7.JPG)

Above is the navigation data message decoded from incoming signal of urban. Comapred to the open-sky, the amplitude is not stable, which means the energy is not concentrated on I-channel, again proving that the received signal is not well tracked;

Key parameters extracted from navigation message.
**Ephemeris Data (31 parameters)**

![image](https://github.com/EinsteinC/AAE6102/blob/main/14.JPG)

## Task 4: Position and velocity estimation
**Weighted least square for positioning**

**Elevation weighted**

```
     weight(i)=sin(el(i))^2;
......
    W=diag(weight);
    C=W'*W;
    x=(A'*C*A)\(A'*C*omc);
```

**Weighted least square for velocity**

```
%===To calculate receiver velocity=====HD
b=[];
lamda=settings.c/1575.42e6;
rate=-lamda*doppler;
rate=rate';
satvelocity=satvelocity';
for i=1:nmbOfSatellites
    b(i)=rate(i)-satvelocity(i,:)*(A(i,1:3))';
end
v=(A'*C*A)\(A'*C*b');
```

### The positioning result of open-air scenario is shown below, where the yellow dot is the ground truth

In open-air environments, the weighted least squares (WLS) solution showcases high accuracy, closely aligning with ground truth measurements. This exceptional precision is primarily due to the lack of prominent signal propagation impairments like multipath interference and non-line-of-sight (NLOS) errors in unobstructed atmospheric conditions.

![image](https://github.com/EinsteinC/AAE6102/blob/main/222.JPG)
![image](https://github.com/EinsteinC/AAE6102/blob/main/333.JPG)

### The positioning result of urban scenario is shown below, where the yellow dot is the ground truth

Urban GNSS positioning suffers from reduced accuracy compared to open environments due to signal obstruction by buildings, multipath reflections, and non-line-of-sight (NLOS) reception, which distort satellite measurements. These challenges degrade geometric diversity (e.g., fewer visible satellites, higher DOP) and introduce meter-level errors. 

![image](https://github.com/EinsteinC/AAE6102/blob/main/555.JPG)
![image](https://github.com/EinsteinC/AAE6102/blob/main/666.JPG)

The Velocity by WLS varies very significantly if no filtering.
## Task 5: Kalman-filter based positioning and velociy
Extended Kalman Filter is applied here
···
**state=[x,y,z,vx,vy,vz,dt,ddt](Position, velocity, clock error and clock drift)**
```
% prediction 
X_kk = F * X;
P_kk = F*P*F'+Q;
...
r = Z - h_x;
S = H * P_kk * H' + R;
K = P_kk * H' /S; % Kalman Gain

% Update State Estimate
X_k = X_kk + (K * r);
I = eye(size(X, 1));
P_k = (I - K * H) * P_kk * (I - K * H)' + K * R * K';
```
In comparison to traditional Weighted Least Squares (WLS) methods, Kalman Filter-based positioning demonstrates smoother trajectories with fewer abrupt jumps or outliers. This improved stability is a result of the Kalman Filter's inherent advantages in dynamic state estimation.

The Kalman Filter provides superior performance over WLS by incorporating temporal continuity, dynamic noise adaptation, and recursive state estimation, leading to more consistent and stable positioning results. Unlike WLS, which treats each epoch independently and is susceptible to jumps caused by measurement noise, the Kalman Filter utilizes a state-space model to propagate estimates forward by considering motion dynamics such as velocity and clock drift. It dynamically adjusts process noise (Q) and measurement noise (R) to suppress outliers and model uncertainties effectively.

### The positioning result of EKF under open air.
![image](https://github.com/EinsteinC/AAE6102/blob/main/777.JPG)

![image](https://github.com/EinsteinC/AAE6102/blob/main/888.JPG)

The velocity after Extended Kalman Filter is alos well improved compared to WLS.

![image](https://github.com/EinsteinC/AAE6102/blob/main/999.JPG)

### The positioning result of EKF under urban area

![image](https://github.com/EinsteinC/AAE6102/blob/main/000.JPG)

![image](https://github.com/EinsteinC/AAE6102/blob/main/0001.JPG)

The velocity after Kalman filter:
![image](https://github.com/EinsteinC/AAE6102/blob/main/0002.JPG)

In contrast to the open-sky environment, the accuracy of both positioning and velocity tends to decrease in environments with obstructions or signal interference. Factors such as multipath effects, non-line-of-sight (NLOS) conditions, and signal blockages can lead to reduced accuracy in determining both position and velocity. These environmental challenges can introduce errors and inaccuracies in the measurements and calculations, impacting the overall precision of the positioning and velocity estimates.
