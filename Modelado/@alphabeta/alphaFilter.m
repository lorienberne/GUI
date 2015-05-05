function med = alphaFilter(this,sigma1,sigma2,medida1,medida2)
  alpha = sigma2^2/(sigma1^2 + sigma1^2);
  med   = [alpha (1-alpha)]*[medida1; medida2];
end
