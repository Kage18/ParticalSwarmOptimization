function [Cost,z]=cost(x,i,nPop)


z=sqrt((x(i).Position(1) - 40).^2 + (x(i).Position(2) - 40).^2);

RelativeDis = 0;
for j = 1:nPop
    
    RelativeDis = RelativeDis + sqrt((x(i).Position(1) - x(j).Position(1)).^2 + (x(i).Position(2) - x(j).Position(2)).^2);
end

RelativeDis = RelativeDis/(nPop-1);
Alpha = 1;
Beta = 4;
Cost =Alpha * z + Beta * (RelativeDis-x(i).Energy);