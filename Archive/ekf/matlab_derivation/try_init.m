%clc;
for cal = 1:length(track(:,1))
    X = [ones(length(track(1:cal,1)),1) track(1:cal,1)];
    b = X \ track(1:cal,2);
        if (track(cal,2)-track(1,2) > 0 && track(cal,1)-track(1,1) < 0) || (track(cal,2)-track(1,2) < 0 && track(cal,1)-track(1,1) < 0)
            psi = pi() + atan(b(2));
        else
            psi = atan(b(2));
        end
    R = 1 - sum((track(1:cal,2) - X*b).^2)/sum((track(1:cal,2) - mean(track(1:cal,2))).^2)
    %rad2deg(psi)
    %slope = rad2deg(atan(b(2)))
end
slope = rad2deg(atan(b(2)))
dydx = (track(end,2)-track(1,2))/(track(end,1)-track(1,1));
rad2deg(atan(dydx))
%rad2deg(atan(b(1,1)))
reg = polyfit(track(:,1), track(:,2), 1);
grad = rad2deg(atan(reg(1)))