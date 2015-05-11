%% QUADCOPTER CLASS
% SIMULATES A QUADCOPTERS MOVEMENT PROVIDED PHYSICAL PARAMETERS AND MOTORS CONTROL SIGNALS

classdef quad
    properties
        % ATTRIBUTE DEFINITION AND PROVISIONAL INITIALIZATION;
        % ACTUAL INTIALIZATION HAPPENS IN THE CONSTRUCTOR;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % CUADCOPTER'S PHYSICAL CARACTERISTICS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        l       = 0;      %DISTANCE BETWEEN THE AXIS OF A PAIR OF PROPELLERS DIVIDED BY TWO
        m       = 0;      %MASS OF THE QUADCOPTER
        rotorIz = 0;      %MOMENT OF INERTIA OF EACH PROPELLER
        inertia = [0 0 0; %TENSOR OF INERTIA OF THE QUADCOPTER
                   0 0 0;
                   0 0 0];
        kProp   = [0 0];  %PROPS CONSTANTS, K_THRUST, K_TORQUE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % MECHANIC'S VARIABLES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        pqr    = zeros(3,1);                  %ANGULAR SPEED FROM BODY REFERENCE FRAME

        attitSttVect = zeros(6,1); %PHI, THETA, PSI, PHIPTO, THETAPTO, PSIPTO;
        posSttVect   = zeros(6,1); %X, Y, Z, XPTO, YPTO, ZPTO;
        r2pto        = zeros(3,1);          %X2PTO, Y2PTO, Z2PTO;

        rotorOmega = [509.35 509.35 509.35 509.35];            %THE ANGULAR SPEED OF THE ROTORS IN RADS/S

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % ROTATION MATRICES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        curLHb = zeros(3,3);  %ROTATION MATRIX LOCAL HORIZON REFERENCE FRAME -> BODY REFERENCE FRAME
        curLH1 = zeros(3,3);  %ROTATION MATRIX LOCAL HORIZON REFERENCE FRAME -> 1 REFERENCE FRAME
        curL12 = zeros(3,3);  %ROTATION MATRIX             1 REFERENCE FRAME -> 2 REFERENCE FRAME
        curL2b = zeros(3,3);  %ROTATION MATRIX             2 REFERENCE FRAME -> BODY REFERENCE FRAME


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % SIMULATION PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        dt;      %THE DURATION IN SECONDS OF EACH TIMESTEP
        qplot;
    end

    methods
        %CONSTRUCTOR -> INTIALICE CLASS ATTRIBUTES
        function this = quad(posSttVect, attitSttVect, m, l, i, rotorIz, kProp, dt)
            this.posSttVect    = posSttVect;
            this.attitSttVect  = attitSttVect;

            this.l       = l;
            this.m       = m;
            this.inertia = [i(1)  0    0 ;
                             0   i(2)  0 ;
                             0    0  i(3)];
            this.rotorIz = rotorIz;
            this.kProp   = kProp;
            this.dt      = dt;
            
        end

        phiOmegaPsi = pqr2phiOmegaPsiPto(pqr,pOp);

        %COMPUTES THE NEXT TIME STEP
        this = simQuad(this);

        %DRAWS THE QUAD IN PLOT FIGURE
        this = drawQuad(this, axScale, ax, fig);

        %GETTERS
        rotOmega = getRotorOmega(this);
        thrust   = getThrust(this);
        torque   = getTorque(this);

        %SETTERS
        %setRotorOmega(rOmega);
    end

end
