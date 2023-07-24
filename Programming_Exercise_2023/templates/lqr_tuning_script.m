params = generate_params;
x0 = params.model.InitialConditionA;

qx = 75:0.5:85; %80:1:100;
qy = 0.11:0.005:0.15; %0.14:0.005:0.16;
qz = 230:5:250; %100:10:500;
qvx = 0.0001:0.001:0.001; %0.001:0.001:0.01;
qvy = 0.001:0.01:0.01; %0.01:0.01:0.1;
qvz = 0.001:0.01:0.01; %0.01:0.01:0.1;

q_next = [94.0;0.1579;300;0.01;0.1;0.1];

[Qz, Qvz] = ndgrid(qz,qvz);
[I,J] = size(Qz);
Q = [qx(1);qy(1);qz(1);qvx(1);qvy(1);qvz(1)].*ones(length(x0), I*J);
idx = 1;
for i=1:I
    for j=1:J
        Q(3, idx) = Qz(i,j);
        Q(6, idx) = Qvz(i,j);
        idx = idx + 1;
    end
end

[~, i_opt] = lqr_tuning(x0,Q,params);

[Qx, Qy, Qvx, Qvy] = ndgrid(qx,qy,qvx,qvy);
[I,J,K,L] = size(Qx);
Q = Q(:, i_opt).*ones(length(x0), I*J*K*L);
idx = 1;
for i=1:I
    for j=1:J
        for k=1:K
            for l=1:L
                Q(1, idx) = Qx(i,j,k,l);
                Q(2, idx) = Qy(i,j,k,l);
                Q(4, idx) = Qvx(i,j,k,l);
                Q(5, idx) = Qvy(i,j,k,l);
                idx = idx + 1;
            end
        end
    end
end

[tuning_struct, i_opt] = lqr_tuning(x0,Q,params);
q = Q(:,i_opt); % optimal q

disp(Q(:,i_opt))

save("lqr_tuning_script.mat", "tuning_struct", "q"); % overwrites
