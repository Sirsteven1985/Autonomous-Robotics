function midPoint = midPointMap(mapStuff)

x1_Map = mapStuff(:,1);
y1_Map = mapStuff(:,2);
x2_Map = mapStuff(:,3);
y2_Map = mapStuff(:,4);

figure
hold on
plot(x1_Map(1:4),y1_Map(1:4),x2_Map(3:4),y2_Map(3:4))
plot(x1_Map(5:8),y1_Map(5:8),x2_Map(5:8),y2_Map(5:8))
plot(x1_Map(9:10),y1_Map(9:10),x2_Map(9:10),y2_Map(9:10))
plot(x1_Map(11:12),y1_Map(11:12),x2_Map(11:12),y2_Map(11:12))
plot(x1_Map(13:14),y1_Map(13:14),x2_Map(13:14),y2_Map(13:14))
plot(x1_Map(15:16),y1_Map(15:16),x2_Map(15:16),y2_Map(15:16))
hold off

for n=1:1:16;
    midLineMap([n,n]) = [((x2_Map(n)-x1_Map(n))/2),((y2_Map(n)-y1_Map(n))/2)];
end
midLineMap = midLineMap';
% Determine none zero points in which a midpoint exists.
k = 1;
for m=1:1:16;
    
    if(midLineMap(5) ~= 0)
        midPxMap_mag([k;1]) = sqrt(((x2_Map(m)-x1_Map(m))/2)^2 + ((y2_Map(m)-y1_Map(m))/2)^2);
        midPxMap_angle([k;1]) = (atan((abs(y2_Map(m)-y1_Map(m))/2)/abs(x2_Map(m)-x1_Map(m)))) + pi;        
       k = k+1;
    end
    
end


midPoint = [midPxMap_mag;midPxMap_angle];
midPoint = midPoint';
end