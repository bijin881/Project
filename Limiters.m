function controls=Limiters(controls, conv)
% Apply Limit and Convert to degrees
pi=atan(1)*4; piby6=pi/6;% pi=3.141592653589793;
rads=pi/180;
%delth=controls(7);
if conv>0
% Convert to degrees
for j=1:6
    controls(j)=controls(j)/rads;
end
end
  for j=1:3
      if abs(controls(j))>30
          controls(j)=30*sign(controls(j));
      end
  end
  if abs(controls(6))>60
      controls(6)=60*sign(controls(6));
  end
  if controls(4)>60
      controls(4)=60;
  end
  if controls(4)<0
      controls(4)=0;
  end
  if controls(5)<-60
      controls(5)=-60;
  end
  if controls(5)>0
      controls(5)=0;
  end
  %controls(7)=delth;
