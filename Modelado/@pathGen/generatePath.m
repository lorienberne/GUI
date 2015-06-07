function this = generatePath(this,points,type)
% Generate a cubic sline from the given points
% Returns the coefficients corresponding to the following expression
% A*t^3 + B*t^2 + C*t + D;
this.points = points;
numPoints = length(points);
unknowns = numPoints*4;
coefs = zeros(3,unknowns);

% B = A*X

for dim = 1:3
    A = zeros(unknowns);
    B = zeros(unknowns,1);
    for i = 1:numPoints
        A(i,4*i) = 1;
        A(i + numPoints,((i-1)*4+1):(i*4)) = ones(1,4);


        A(i + 2*numPoints,((i-1)*4+1)) = -3;
        A(i + 2*numPoints,((i-1)*4+2)) = -2;
        A(i + 2*numPoints,((i-1)*4+3)) = -1;
        
        
        A(i + 3*numPoints,((i-1)*4+1)) = -6;
        A(i + 3*numPoints,((i-1)*4+2)) = -2;
        
        B(i,1) = points(i,dim);

        
        if (~(i == numPoints))
            A(i + 2*numPoints,((i-1)*4+7)) =  1;
            A(i + 3*numPoints,((i-1)*4+6)) =  2;
            if(strcmp(type(i),'Fly-Over'))
                B(i + numPoints,1) = points(i+1,dim);
            else
            end
        else
            A(i + 2*numPoints,3) =  1;
            A(i + 3*numPoints,2) =  2;
            if(strcmp(type(i),'Fly-Over'))
                B(i + numPoints,1) = points(1,dim);    
            else
            end
        end
        
    end
    
    coefs(dim,:) = linsolve(A,B)';
    this.coefs = coefs;
end

end
