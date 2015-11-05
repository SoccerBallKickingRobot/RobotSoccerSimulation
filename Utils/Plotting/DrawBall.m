function DrawBall(Ball)
surf(Ball.dims.radius*Ball.dims.x + Ball.states(1),...
    Ball.dims.radius*Ball.dims.y + Ball.states(2),...
    Ball.dims.radius*Ball.dims.z + Ball.states(3),...
    Ball.color.values);
%shading interp
XL = get(gca, 'XLim');
YL = get(gca, 'YLim');

patch([XL(1), XL(2), XL(2), XL(1)], [YL(1), YL(1), YL(2), YL(2)], [0 0 0  0], 'FaceColor', 0.8*[0 1 0]);