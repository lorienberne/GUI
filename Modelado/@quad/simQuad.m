function this = simQuad(this)
    %UPDATE ROTATION MATRICES
    [this.curLHb, this.curLH1, this.curL12, this.curL2b] = calcRotMat(this.attitSttVect(1:3));

    %CALCULATE THE THRUST FOR EACH PROPELLER
    T = this.getThrust();

    %CALCULATE THE A VECTOR RESULT OF THE SUM OF ALL EXISTING TORQUES
    M = this.getTorque();

    %CALCULATE THE ANGULAR ACCELERATION IN THE BODY REFERENCE FRAME
    pqrPto = this.inertia\(M - cross(this.pqr, this.inertia*this.pqr) - cross(this.pqr, [0; 0; (this.rotorIz * (-this.rotorOmega(1) + this.rotorOmega(2) - this.rotorOmega(3) + this.rotorOmega(4)))]));

    %CALCULATE THE ANGULAR SPEED
    this.pqr = this.pqr + pqrPto * this.dt;

    %CHANGE THE ANGULAR SPEED TO A ABSOLUTE REFERENCE FRAME
    this.attitSttVect(4:6,1) = this.pqr2phiThetaPsiPto(this.pqr, this.attitSttVect(1:3));

    %CALCULATE THE CURRENT ATTITUDE
    this.attitSttVect(1:3,1) = this.attitSttVect(1:3) + this.attitSttVect(4:6) * this.dt;


    %CALCULATE THE ACCELERATION OF THE QUADCOPTER AND SAVE IT FOR USE IN OTHER CLASSES (EX. ACCELEROMETER CLASS)
    this.r2pto = (1/this.m)*(this.curLHb\[0; 0; (-T(1) -T(2) - T(3) - T(4))]) + [0; 0; 9.81];

    %UPDATE POSITION STATE VECTOR
    this.posSttVect(4:6,1) = this.posSttVect(4:6,1) + this.r2pto * this.dt; %UPDATE SPEED
    this.posSttVect(1:3,1) = this.posSttVect(1:3,1) + this.posSttVect(4:6,1) * this.dt;%UPDATE POSITION
end
