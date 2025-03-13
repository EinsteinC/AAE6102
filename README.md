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



### Acquistion result from urban

** Four Satellites are acquired**

Comapred to open-sky environments, less GPS satellites are acquired due to the blockage of buildings.

## Task 2: Tracking
### 2.1 Code Analysis
The motivation of tracking is to refine carrier frequency and code phase values, keep track.  

Code tracking: early,late prompt discriminator with spacing 0.5 chips

Calculate the correlator output
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

The Q-channel tends to zero: Ideally, the Q-channel contains only noise and residual error, and its value fluctuates around zero， which verifies the carrier phase is aligned.


DLL output (Code Discriminator Output) near zero: indicates that the local code is aligned with the code phase of the received signal and the code tracking error is minimal.

PLL output (Phase/Frequency Discriminator Output) near zero: indicates that the local carrier is synchronized with the received signal carrier and the carrier tracking is stable.


The Prompt correlation is significantly greater than Early/Late, indicating that the code phases are precisely aligned and the DLL is in a stable tracking state.

**Autocorrelation Function from Multi-correlator output**

The shape of ACF is symmetric and undistorted, indicating the satellite is undistorted and not under the influence of multipath, which is in agreement with the experiment scenario (Open Sky);
**Above proves that the satellite in open sky is well acquired and tracked**

### 2,4 Tracking result from data urban （PRN18 in urban as an example）


The Q-channel is not always about zero, sometimes even greater than I-channel, which means not all energy are concentrated on I-channel, the carrier phase is not always aligned well


DLL output (Code Discriminator Output) is similar to that in open-sky.

The PLL output is not always near zero with great increase during sometimes, indicates that the local carrier is not always synchronized with the received signal carrier and the carrier tracking is not stable.


The Prompt correlation is not always significantly greater than Early/Late, sometimes even weaker (**when PLL is greater**), indicating that the carrier phases are not precisely aligned.


Multi-correaltor output is not symmetric, which means multipath distorts the shape of ACF, which will lead to incorrect pseudorange measurement and consequently wore positioning accuracy.

**Above proves that the satellite in urban is not well acquired and tracked**

## Task 3: Navigation data decoding (PRN 16 Open-Sky and PRN18 urban as an Example)

Above is the navigation data message decoded from incoming signal of open sky.


Above is the navigation data message decoded from incoming signal of urban. Comapred to the open-sky, the amplitude is not stable, which means the energy is not concentrated on I-channel, again proving that the received signal is not well tracked;

Key parameters extracted from navigation message.
**Ephemeris Data (31 parameters)**

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

The weighted least squares (WLS) solution demonstrates **high accuracy** in open-air environments, exhibiting close alignment with ground truth measurements. This observed precision can be attributed to the absence of significant signal propagation impairments such as multipath interference and non-line-of-sight (NLOS) errors under unobstructed atmospheric conditions.

### The positioning result of urban scenario is shown below, where the yellow dot is the ground truth

Urban GNSS positioning suffers from reduced accuracy compared to open environments due to signal obstruction by buildings, multipath reflections, and non-line-of-sight (NLOS) reception, which distort satellite measurements. These challenges degrade geometric diversity (e.g., fewer visible satellites, higher DOP) and introduce meter-level errors. 


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
Compared to traditional Weighted Least Squares (WLS), Kalman Filter-based positioning exhibits smoother trajectories with fewer abrupt jumps or outliers. This enhanced stability stems from the Kalman Filter’s inherent advantages in dynamic state estimation.
The Kalman Filter offers superior performance over Weighted Least Squares (WLS) by integrating temporal continuity, dynamic noise adaptation, and recursive state estimation, resulting in smoother, more stable positioning. Unlike WLS, which processes each epoch independently and is prone to measurement noise-induced jumps, the Kalman Filter leverages a state-space model to propagate estimates forward using motion dynamics (velocity and clock drift), while dynamically balancing process noise (Q) and measurement noise (R) to suppress outliers and model uncertainties. 

### The positioning result of EKF under open air.

The velocity after Extended Kalman Filter is alos well improved compared to WLS.


### The positioning result of EKF under urban area

The velocity after Kalman filter:

Compared to the open-sky environment, the positioning and velocity are both not so accurate.

