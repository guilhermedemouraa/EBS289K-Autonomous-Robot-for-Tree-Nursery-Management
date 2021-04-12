function draw_disc(xc, yc, radius, R, C)
global bitmap;

for ii = xc-int16(radius):xc+(int16(radius)) 
for jj = yc-int16(radius):yc+(int16(radius)) 
tempR = sqrt((double(ii) - double(xc)).^2 + (double(jj) - double(yc)).^2); 
if(tempR <= double(int16(radius))) 
bitmap(R-ii,jj)=1; 
end 
end 
end

end

