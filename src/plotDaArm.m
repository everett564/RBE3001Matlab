function T = plotDaArm(q)
    
    %hold on;
    
    [p1 p2 p3] = fwkin3001(q(1), q(2), q(3));
    
    arm1 = line(nan, nan, 'color', 'r'); 
    arm2 = line(nan, nan, 'color', 'g');
    arm3 = line(nan, nan, 'color', 'b');
    
    arm1 = [(p1(1)-0),(p1(2)-0),(p2(3)-0)]
    arm2 = [(p2(1)-p1(1)),(p2(2)-p1(2)),(p2(3)-p1(3))]
    arm2 = [(p3(1)-p2(1)),(p3(2)-p2(2)),(p3(3)-p2(3))]
%     set(hx, 'XData', p1(1) , 'YData', p1(2) ,'ZData', p(3));
%     set(hy, 'XData', , 'YData', ,'ZData', );
%     set(hz, 'XData', , 'YData', ,'ZData', );
    plot3([0, p1(1), p2(1), p3(1)], [0, p1(2), p2(2), p3(2)], [0, p1(3), p2(3), p3(3)],'-or');


    xlim([0,350])
    ylim([-200,200])
    zlim([-50,300])%plot3(arm2);
    pause (.003);
    
    

end