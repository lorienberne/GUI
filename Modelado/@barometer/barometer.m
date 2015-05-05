classdef barometer
    properties
        cov;
    end
    methods
        function this = barometer(cov)
            this.cov = cov;
        end
        alt = baroMeasure(this,quad);
    end
end