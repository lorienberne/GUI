%%  Kalman Filter Class
% Creates a Kalman Filter object from Q and R matrices
%
% R MATRIX MUST BE ARRANGED AS FOLLOWS
% R = [M 0 0 0 0 0;   M -> MAGNETOMETER'S COVARIANCE
%      0 A 0 0 0 0;   A -> ACCELEROMETER'S COVARIANCE
%      0 0 A 0 0 0;
%      0 0 0 G 0 0;   G -> GYROSCOPE'S COVARIANCE
%      0 0 0 0 G 0;
%      0 0 0 0 0 G];


classdef kalman
    properties
        xPriori;
        xPosteriori;
        pPriori;
        pPosteriori;
        K;

% COVARIANCE MATRICES
        Q;
        R;

% MODEL MATRICES

        A;
        B;
        C;
        D;

% SENSORS OBJECTS
        dt;

    end

    methods
        function this = kalman(Q,R,A,B,C,D,dt)
        %% CONSTRUCTOR. INTIALIZATION OF VARIABLES
            this.Q      = Q;
            this.R      = R;
            this.A      = A;
            this.B      = B;
            this.C      = C;
            this.D      = D;
            this.dt     = dt;
            this.xPriori     = zeros(length(A),1);
            this.xPosteriori = zeros(length(A),1);
            this.pPriori     = zeros(length(A),length(A));
            this.pPosteriori = zeros(length(A),length(A));
            this.K           = zeros(length(A),1);
        end

        this = CalcXPriori(this,u);
        this = CalcXPosteriori(this, mState);
        this = CalcPPriori(this);
        this = CalcPPosteriori(this);
        this = CalcK(this);
        this = updateKalman(this,quad, mState);

        %GETERS
        state = getState(this);
    end
end
