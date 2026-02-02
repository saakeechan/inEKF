% clear all; close all; clc;
% 
% addpath("./../include");
% cd('./fig_est_vs_window/result_0.5_0.1');
% %./fig_est_vs_window/result_backppfrom1_updvO
% slip_mode = 1;
% if(slip_mode)
%     slip = 0.48;
%     SRmode=['(SR' num2str(slip) ')'];
% else
%     slip=0.48;
%     SRmode='';
% end
% 
% ret_mode = '(RetX)';
% 
% cov='-3-1-1.3-5-6 -10 -10-10';
% initial = '0.00_0.00_0.00_0.00_0.00_0.00_0.00_0.00_0.00_0.00_0.00_0.00';
% % 0.00_0.00_0.00_0.00_0.00_0.00_0.00_0.00_0.00_0.00_0.00_0.00
% data='20220623_IS_04x30000_';
% %data1 = '20220623_IS_04x25000_';
% window='1';
% 
% slip_offset = 17;
% 
% %num2str(10)
% result1 = readtable([SRmode ret_mode '(0.001_1)(0.7_1000)AIS_(' cov ')_(' initial ')_' data num2str(1) '.txt']);
% name1 = 'AIS(it1)';
% 
% result2=result1; result3=result2; result4=result3; result5=result4;
% name2=name1; name3=name2; name4=name3; name5=name4;
% 
% result2 = readtable([SRmode ret_mode '(0.001_1)(0.7_1000)PIS_(' cov ')_(' initial ')_' data num2str(1) '.txt']);
% name2 = 'PIS(it10)';
% 
% result3 = readtable([SRmode ret_mode '(0.001_1)(0.7_1000)IS_(' cov ')_(' initial ')_' data num2str(1) '.txt']);                                                                                        
% name3 = 'IS(it1)';
% 
% result4 = readtable([SRmode ret_mode '(0.001_1)(0.7_1000)NIS_(' cov ')_(' initial ')_' data num2str(1) '.txt']);
% name4 = 'NIS(it10)';
% 
% %result4 = readtable([SRmode ret_mode '(0.001_1)(0.7_10)PI_(' cov ')_(' initial ')_' data num2str(5) '.txt']);
% %name4 = 'H';
% 
% %result4 = readtable(['H(adj)_(' cov ')_(' initial ')_' data '1.txt']);
% result5 = readtable([SRmode 'H(adj)_(' cov ')_(' initial ')_' data '1.txt']);
% name5 = 'H SR';
% 

%%

clear; close all; clc;

% --- Base folder in MATLAB Drive ---
% baseFolder = fullfile('MATLAB Drive', 'Research', 'dataFiles');

cd result/

% --- File information ---
slip = 0.3;
SRmode = sprintf('(SR%.1f)', slip);
cov = '-5-1-1-4-8 -10 -10-10';
initial = '0.00_0.00_0.00_0.00_0.00_0.00_0.00_0.00_0.00_0.00_0.00_0.00';
data = '20220623_IEKF_01x27000_';
index = 1;

% --- Construct filename and full path ---
fileName = [SRmode 'H(adj)_(' cov ')_(' initial ')_' data num2str(index) '.txt'];
filePath = fileName;


% --- Read the file ---
if isfile(filePath)
    result1 = readtable(filePath);
    result2 =  readtable(filePath);
    result3 = readtable(filePath);
    result4 = readtable(filePath);
    result5 = readtable(filePath);
    disp('File loaded successfully!');
else
    error('File not found:\n%s', filePath);
end

% --- Label for plotting or legend use ---
name1 = 'H SR (Your File)';
name2 = 'H SR (Your File)';
name3 = 'H SR (Your File)';
name4 = 'H SR (Your File)';
name5 = 'H SR (Your File)';







%% Basic

[finish,b] = size(result1);

dt=0.005;
finish=(120/dt);
start=(0/dt)+1;
t = start:1:finish-1;
t = t.*dt;

for  k=start:1:finish-1
    
    v1_rel_true(k, 1:3) = ([result1.Var1(k), result1.Var2(k), result1.Var3(k); result1.Var4(k), result1.Var5(k), result1.Var6(k); result1.Var7(k), result1.Var8(k), result1.Var9(k)]' * [result1.Var10(k); result1.Var11(k); result1.Var12(k)])';
    
    v1_rel_est(k, 1:3) = ([result1.Var49(k), result1.Var50(k), result1.Var51(k); result1.Var52(k), result1.Var53(k), result1.Var54(k); result1.Var55(k), result1.Var56(k), result1.Var57(k)]' * [result1.Var58(k); result1.Var59(k); result1.Var60(k)])' ;
    
end

for  k=start:1:finish-1
    
    v2_rel_true(k, 1:3) = ([result2.Var1(k), result2.Var2(k), result2.Var3(k); result2.Var4(k), result2.Var5(k), result2.Var6(k); result2.Var7(k), result2.Var8(k), result2.Var9(k)]' * [result2.Var10(k); result2.Var11(k); result2.Var12(k)])';
    
    v2_rel_est(k, 1:3) = ([result2.Var49(k), result2.Var50(k), result2.Var51(k); result2.Var52(k), result2.Var53(k), result2.Var54(k); result2.Var55(k), result2.Var56(k), result2.Var57(k)]' * [result2.Var58(k); result2.Var59(k); result2.Var60(k)])' ;
    
end

for  k=start:1:finish-1
    
    v3_rel_true(k, 1:3) = ([result3.Var1(k), result3.Var2(k), result3.Var3(k); result3.Var4(k), result3.Var5(k), result3.Var6(k); result3.Var7(k), result3.Var8(k), result3.Var9(k)]' * [result3.Var10(k); result3.Var11(k); result3.Var12(k)])';
    
    v3_rel_est(k, 1:3) = ([result3.Var49(k), result3.Var50(k), result3.Var51(k); result3.Var52(k), result3.Var53(k), result3.Var54(k); result3.Var55(k), result3.Var56(k), result3.Var57(k)]' * [result3.Var58(k); result3.Var59(k); result3.Var60(k)])' ;
    
end

for  k=start:1:finish-1
    
    v4_rel_true(k, 1:3) = ([result4.Var1(k), result4.Var2(k), result4.Var3(k); result4.Var4(k), result4.Var5(k), result4.Var6(k); result4.Var7(k), result4.Var8(k), result4.Var9(k)]' * [result4.Var10(k); result4.Var11(k); result4.Var12(k)])';
    
    v4_rel_est(k, 1:3) = ([result4.Var49(k), result4.Var50(k), result4.Var51(k); result4.Var52(k), result4.Var53(k), result4.Var54(k); result4.Var55(k), result4.Var56(k), result4.Var57(k)]' * [result4.Var58(k); result4.Var59(k); result4.Var60(k)])' ;
    
end

for  k=start:1:finish-1
    
    v5_rel_true(k, 1:3) = ([result5.Var1(k), result5.Var2(k), result5.Var3(k); result5.Var4(k), result5.Var5(k), result5.Var6(k); result5.Var7(k), result5.Var8(k), result5.Var9(k)]' * [result5.Var10(k); result5.Var11(k); result5.Var12(k)])';
    
    v5_rel_est(k, 1:3) = ([result5.Var49(k), result5.Var50(k), result5.Var51(k); result5.Var52(k), result5.Var53(k), result5.Var54(k); result5.Var55(k), result5.Var56(k), result5.Var57(k)]' * [result5.Var58(k); result5.Var59(k); result5.Var60(k)])' ;
    
end

for  k=start:1:finish-1
    
    rot_error1(k, 1:3)  = rotm2eul( [result1.Var1(k), result1.Var2(k), result1.Var3(k); result1.Var4(k), result1.Var5(k), result1.Var6(k); result1.Var7(k), result1.Var8(k), result1.Var9(k)]' * [result1.Var49(k), result1.Var50(k), result1.Var51(k); result1.Var52(k), result1.Var53(k), result1.Var54(k); result1.Var55(k), result1.Var56(k), result1.Var57(k)]);
    phi_error1(k,1:3)   = logm_vec( [result1.Var49(k), result1.Var50(k), result1.Var51(k); result1.Var52(k), result1.Var53(k), result1.Var54(k); result1.Var55(k), result1.Var56(k), result1.Var57(k)] ) - logm_vec( [result1.Var1(k), result1.Var2(k), result1.Var3(k); result1.Var4(k), result1.Var5(k), result1.Var6(k); result1.Var7(k), result1.Var8(k), result1.Var9(k)] );
    Hrpy_error1(k,1:3) = Rotation_to_Euler( [result1.Var49(k), result1.Var50(k), result1.Var51(k); result1.Var52(k), result1.Var53(k), result1.Var54(k); result1.Var55(k), result1.Var56(k), result1.Var57(k)] ) - Rotation_to_Euler( [result1.Var1(k), result1.Var2(k), result1.Var3(k); result1.Var4(k), result1.Var5(k), result1.Var6(k); result1.Var7(k), result1.Var8(k), result1.Var9(k)] );
    
    rot_error2(k, 1:3)  = rotm2eul( [result2.Var1(k), result2.Var2(k), result2.Var3(k); result2.Var4(k), result2.Var5(k), result2.Var6(k); result2.Var7(k), result2.Var8(k), result2.Var9(k)]' * [result2.Var49(k), result2.Var50(k), result2.Var51(k); result2.Var52(k), result2.Var53(k), result2.Var54(k); result2.Var55(k), result2.Var56(k), result2.Var57(k)]);
    phi_error2(k,1:3)   = logm_vec( [result2.Var49(k), result2.Var50(k), result2.Var51(k); result2.Var52(k), result2.Var53(k), result2.Var54(k); result2.Var55(k), result2.Var56(k), result2.Var57(k)] ) - logm_vec( [result2.Var1(k), result2.Var2(k), result2.Var3(k); result2.Var4(k), result2.Var5(k), result2.Var6(k); result2.Var7(k), result2.Var8(k), result2.Var9(k)] );
    Hrpy_error2(k,1:3) = Rotation_to_Euler( [result2.Var49(k), result2.Var50(k), result2.Var51(k); result2.Var52(k), result2.Var53(k), result2.Var54(k); result2.Var55(k), result2.Var56(k), result2.Var57(k)] ) - Rotation_to_Euler( [result2.Var1(k), result2.Var2(k), result2.Var3(k); result2.Var4(k), result2.Var5(k), result2.Var6(k); result2.Var7(k), result2.Var8(k), result2.Var9(k)] );
    
    rot_error3(k, 1:3)  = rotm2eul( [result3.Var1(k), result3.Var2(k), result3.Var3(k); result3.Var4(k), result3.Var5(k), result3.Var6(k); result3.Var7(k), result3.Var8(k), result3.Var9(k)]' * [result3.Var49(k), result3.Var50(k), result3.Var51(k); result3.Var52(k), result3.Var53(k), result3.Var54(k); result3.Var55(k), result3.Var56(k), result3.Var57(k)]);
    phi_error3(k,1:3)   = logm_vec( [result3.Var49(k), result3.Var50(k), result3.Var51(k); result3.Var52(k), result3.Var53(k), result3.Var54(k); result3.Var55(k), result3.Var56(k), result3.Var57(k)] ) - logm_vec( [result3.Var1(k), result3.Var2(k), result3.Var3(k); result3.Var4(k), result3.Var5(k), result3.Var6(k); result3.Var7(k), result3.Var8(k), result3.Var9(k)] );
    Hrpy_error3(k,1:3) = Rotation_to_Euler( [result3.Var49(k), result3.Var50(k), result3.Var51(k); result3.Var52(k), result3.Var53(k), result3.Var54(k); result3.Var55(k), result3.Var56(k), result3.Var57(k)] ) - Rotation_to_Euler( [result3.Var1(k), result3.Var2(k), result3.Var3(k); result3.Var4(k), result3.Var5(k), result3.Var6(k); result3.Var7(k), result3.Var8(k), result3.Var9(k)] );
    
    rot_error4(k, 1:3)  = rotm2eul( [result4.Var1(k), result4.Var2(k), result4.Var3(k); result4.Var4(k), result4.Var5(k), result4.Var6(k); result4.Var7(k), result4.Var8(k), result4.Var9(k)]' * [result4.Var49(k), result4.Var50(k), result4.Var51(k); result4.Var52(k), result4.Var53(k), result4.Var54(k); result4.Var55(k), result4.Var56(k), result4.Var57(k)]);
    phi_error4(k,1:3)   = logm_vec( [result4.Var49(k), result4.Var50(k), result4.Var51(k); result4.Var52(k), result4.Var53(k), result4.Var54(k); result4.Var55(k), result4.Var56(k), result4.Var57(k)] ) - logm_vec( [result4.Var1(k), result4.Var2(k), result4.Var3(k); result4.Var4(k), result4.Var5(k), result4.Var6(k); result4.Var7(k), result4.Var8(k), result4.Var9(k)] );
    Hrpy_error4(k,1:3) = Rotation_to_Euler( [result4.Var49(k), result4.Var50(k), result4.Var51(k); result4.Var52(k), result4.Var53(k), result4.Var54(k); result4.Var55(k), result4.Var56(k), result4.Var57(k)] ) - Rotation_to_Euler( [result4.Var1(k), result4.Var2(k), result4.Var3(k); result4.Var4(k), result4.Var5(k), result4.Var6(k); result4.Var7(k), result4.Var8(k), result4.Var9(k)] );
    
    rot_error5(k, 1:3)  = rotm2eul( [result5.Var1(k), result5.Var2(k), result5.Var3(k); result5.Var4(k), result5.Var5(k), result5.Var6(k); result5.Var7(k), result5.Var8(k), result5.Var9(k)]' * [result5.Var49(k), result5.Var50(k), result5.Var51(k); result5.Var52(k), result5.Var53(k), result5.Var54(k); result5.Var55(k), result5.Var56(k), result5.Var57(k)]);
    phi_error5(k,1:3)   = logm_vec( [result5.Var49(k), result5.Var50(k), result5.Var51(k); result5.Var52(k), result5.Var53(k), result5.Var54(k); result5.Var55(k), result5.Var56(k), result5.Var57(k)] ) - logm_vec( [result5.Var1(k), result5.Var2(k), result5.Var3(k); result5.Var4(k), result5.Var5(k), result5.Var6(k); result5.Var7(k), result5.Var8(k), result5.Var9(k)] );
    Hrpy_error5(k,1:3) = Rotation_to_Euler( [result5.Var49(k), result5.Var50(k), result5.Var51(k); result5.Var52(k), result5.Var53(k), result5.Var54(k); result5.Var55(k), result5.Var56(k), result5.Var57(k)] ) - Rotation_to_Euler( [result5.Var1(k), result5.Var2(k), result5.Var3(k); result5.Var4(k), result5.Var5(k), result5.Var6(k); result5.Var7(k), result5.Var8(k), result5.Var9(k)] );
    
end

d1_v_1 = sqrt(result1.Var64(1:(finish-1)).^2+result1.Var65(1:(finish-1)).^2+result1.Var66(1:(finish-1)).^2);
d2_v_1 = sqrt(result1.Var67(1:(finish-1)).^2+result1.Var68(1:(finish-1)).^2+result1.Var69(1:(finish-1)).^2);
d3_v_1 = sqrt(result1.Var70(1:(finish-1)).^2+result1.Var71(1:(finish-1)).^2+result1.Var72(1:(finish-1)).^2);
d4_v_1 = sqrt(result1.Var73(1:(finish-1)).^2+result1.Var74(1:(finish-1)).^2+result1.Var75(1:(finish-1)).^2);

idx=0;
for i=start:1:(finish-1)
        
    if( (d1_v_1(i)>slip && result1.Var42(i)) || (d2_v_1(i)>slip && result1.Var43(i)) || (d3_v_1(i)>slip && result1.Var44(i)) || (d4_v_1(i)>slip && result1.Var45(i)))
        
        idx = idx+1;

        slip_pt_1(1,idx) = i*dt;
        bar_height_plus_1(1,idx) = 1;
        bar_height_minus_1(1,idx) = -1;
        
    end
end

d1_v_2 = sqrt(result2.Var64(1:(finish-1)).^2+result2.Var65(1:(finish-1)).^2+result2.Var66(1:(finish-1)).^2);
d2_v_2 = sqrt(result2.Var67(1:(finish-1)).^2+result2.Var68(1:(finish-1)).^2+result2.Var69(1:(finish-1)).^2);
d3_v_2 = sqrt(result2.Var70(1:(finish-1)).^2+result2.Var71(1:(finish-1)).^2+result2.Var72(1:(finish-1)).^2);
d4_v_2 = sqrt(result2.Var73(1:(finish-1)).^2+result2.Var74(1:(finish-1)).^2+result2.Var75(1:(finish-1)).^2);

idx=0;
for i=start:1:(finish-1)
        
    if( (d1_v_2(i)>slip && result2.Var42(i)) || (d2_v_2(i)>slip && result2.Var43(i)) || (d3_v_2(i)>slip && result2.Var44(i)) || (d4_v_2(i)>slip && result2.Var45(i)))
        
        idx = idx+1;

        slip_pt_2(1,idx) = i*dt;
        bar_height_plus_2(1,idx) = 1;
        bar_height_minus_2(1,idx) = -1;
        
    end
end
%slip_pt_2 = slip_pt_2.*dt;


%% thereshold slip phase method
threshold = 0.4;

idx=0;
phase_count_1 = 0; phase_count_2 = 0; phase_count_3 = 0; phase_count_4 = 0;
slip_count_1 = 0; slip_count_2 = 0; slip_count_3 = 0; slip_count_4 = 0;
slip_start_index_1 = 0; slip_start_index_2 = 0; slip_start_index_3 = 0; slip_start_index_4 = 0;
slip_pt(1,1) = 0;
for index=1:size(t,2)
        
    if( result1.Var42(index) )
        
        if(phase_count_1==0)
            slip_start_index_1 = index;
        end
        
        phase_count_1 = phase_count_1+1;
        
        if( d1_v_1(index)>slip )

            slip_count_1 = slip_count_1 + 1;
        end
        
    else
        
        if( phase_count_1>0 && (slip_count_1/phase_count_1)>threshold )
            
            for p=1:phase_count_1
                
                if (slip_start_index_1+(p-1) > slip_pt(1,end))
                    idx = idx+1;
                    slip_pt(1,idx) = slip_start_index_1+(p-1);
                    bar_height_plus(1,idx) = 1;
                    bar_height_minus(1,idx) = -1;
                end
                
            end
        end
        
        slip_flag_1 = false; phase_count_1=0; slip_count_1 = 0;
    end
    
    
    if( result1.Var43(index) )
        
        if(phase_count_2==0)
            slip_start_index_2 = index;
        end
        
        phase_count_2 = phase_count_2+1;
        
        if( d2_v_1(index)>slip )

            slip_count_2 = slip_count_2 + 1;
        end
        
    else
        
        if( phase_count_2>0 && (slip_count_2/phase_count_2)>threshold )
            
            for p=1:phase_count_2
                
                if (slip_start_index_2+(p-1) > slip_pt(1,end))
                    idx = idx+1;
                    slip_pt(1,idx) = slip_start_index_2+(p-1);
                    bar_height_plus(1,idx) = 1;
                    bar_height_minus(1,idx) = -1;
                end
                
            end
        end
        
        slip_flag_2 = false; phase_count_2=0; slip_count_2 = 0;
    end
    
    if( result1.Var44(index) )
        
        if(phase_count_3==0)
            slip_start_index_3 = index;
        end
        
        phase_count_3 = phase_count_3+1;
        
        if( d3_v_1(index)>slip )

            slip_count_3 = slip_count_3 + 1;
        end
        
    else
        
        if( phase_count_3>0 && (slip_count_3/phase_count_3)>threshold )
            
            for p=1:phase_count_3
                
                if (slip_start_index_3+(p-1) > slip_pt(1,end))
                    idx = idx+1;
                    slip_pt(1,idx) = slip_start_index_3+(p-1);
                    bar_height_plus(1,idx) = 1;
                    bar_height_minus(1,idx) = -1;
                end
                
            end
        end
        
        slip_flag_3 = false; phase_count_3=0; slip_count_3 = 0;
    end
    
    if( result1.Var45(index) )
        
        if(phase_count_4==0)
            slip_start_index_4 = index;
        end
        
        phase_count_4 = phase_count_4+1;

        if( d4_v_1(index)>slip )

            slip_count_4 = slip_count_4 + 1;
        end
        
    else
        
        if( phase_count_4>0 && (slip_count_4/phase_count_4)>threshold )
            
            for p=1:phase_count_4
                
                if (slip_start_index_4+(p-1) > slip_pt(1,end))
                    idx = idx+1;
                    slip_pt(1,idx) = slip_start_index_4+(p-1);
                    bar_height_plus(1,idx) = 1;
                    bar_height_minus(1,idx) = -1;
                end
                
            end
        end
        
        slip_flag_4 = false; phase_count_4=0; slip_count_4 = 0;
    end
    
end
slip_pt = slip_pt.*dt;

%slip_pt_1 = slip_pt;
%bar_height_plus_1 = bar_height_plus;
%bar_height_minus_1 = bar_height_minus;
%% Correlation
% figure(30);
% hold on;
% 
% [c,lags]=xcorr(result1.Var94(start:1:a-1), result2.Var94(start:1:a-1), 'Normalized');
% stem(lags,c);
% 
% [c2,lags2]=xcorr(result1.Var94(start:1:a-1), result4.Var94(start:1:a-1), 'Normalized');
% stem(lags2,c2);
% 
% [c3,lags3]=xcorr(result2.Var94(start:1:a-1), result4.Var94(start:1:a-1), 'Normalized');
% stem(lags3,c3);
% legend('IS-PIS', 'IS-NIS', 'PIS-NIS')
% grid minor;
%% dv

% figure(12);
% set(gcf,'units','normalized','outerposition',[0 0 1 1]);
% %plot(t, d1_v_1,  t, d2_v_1,  t, d3_v_1,  t, d4_v_1);
% hold on;
% grid minor;
% legend('d1', 'd2', 'd3', 'd4');
% b1 = bar(slip_pt_1(1,:),bar_height_plus_1(1,:),1);
% set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');
% b1 = bar(slip_pt_1(1,:),bar_height_minus_1(1,:),1);
% set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');
% 
% b2 = bar(slip_pt_2(1,:),bar_height_plus_2(1,:),1);
% set(b2,'FaceAlpha',0.2, 'FaceColor',[0 0 1], 'Linestyle','none');
% b2 = bar(slip_pt_2(1,:),bar_height_minus_2(1,:),1);
% set(b2,'FaceAlpha',0.2, 'FaceColor',[0 0 1], 'Linestyle','none');

figure(12);
clf;
set(gcf,'units','normalized','outerposition',[0 0 1 1]);
hold on; grid minor;

plot(t, d1_v_1, 'k'); % your actual data curve
legend('d1 velocity');

% Example: highlight slip intervals (t_start, t_end)
for i = 1:length(slip_pt_1)
    t_start = slip_pt_1(i) - 0.01; % small width
    t_end   = slip_pt_1(i) + 0.01;
    fill([t_start t_end t_end t_start], [ylim fliplr(ylim)], ...
         'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
end


%% rel velocity


figure(2);
set(gcf,'units','normalized','outerposition',[0 0 1 1]);

subplot(3,1,1);
plot(t, v1_rel_true(start:1:a-1,1), 'k', t, v1_rel_est(start:1:a-1,1), '-.b'); 
title('Vx_rel'); xlabel('Time(s)'); ylabel('velocity(m/s)'); legend('Vx_ true', 'Vx_ estimated'); grid minor;

subplot(3,1,2);
plot(t, v1_rel_true(start:1:a-1,2), 'k', t, v1_rel_est(start:1:a-1,2), '-.b');
title('Vy_rel'); xlabel('Time(s)'); ylabel('velocity(m/s)');legend('Vy_ true', 'Vy_ estimated'); grid minor;

subplot(3,1,3);
plot(t, v1_rel_true(start:1:a-1,3), 'k', t, v1_rel_est(start:1:a-1,3), '-.b');
title('Vz_rel'); xlabel('Time(s)'); ylabel('velocity(m/s)');legend('Vz_ true', 'Vz_ estimated'); grid minor;



%% bias
figure(1);
set(gcf,'units','normalized','outerposition',[0 0 1 1]);

subplot(3,1,1);
plot(t, result1.Var76(start:1:finish-1), 'r', t, result1.Var79(start:1:finish-1), '-.b',t, result4.Var76(start:1:finish-1), 'm', t, result4.Var79(start:1:finish-1), '-.k'); 
title('bx'); xlabel('Time(s)'); ylabel('bias'); legend('bg_x', 'ba_x', 'bg_x Inekf', 'ba_x Inekf'); grid minor;

subplot(3,1,2);
plot(t, result1.Var77(start:1:finish-1),'r', t, result1.Var80(start:1:finish-1), '-.b',t, result4.Var77(start:1:finish-1), 'm', t, result4.Var80(start:1:finish-1), '-.k');
title('by'); xlabel('Time(s)'); ylabel('bias');legend('bg_y', 'ba_y', 'bg_y Inekf', 'ba_y Inekf'); grid minor;

subplot(3,1,3);
plot(t, result1.Var78(start:1:finish-1), 'r',t, result1.Var81(start:1:finish-1), '-.b',t, result4.Var78(start:1:finish-1), 'm', t, result4.Var81(start:1:finish-1), '-.k');
title('bz'); xlabel('Time(s)'); ylabel('bias');legend('bg_z', 'ba_z', 'bg_z Inekf', 'ba_z Inekf'); grid minor;

a = finish - start + 1; % Adjusted definition to represent the number of samples

%% velocity
figure(20);
set(gcf,'units','normalized','outerposition',[0 0 1 1]);

subplot(3,1,1);
plot(t, result1.Var10(start:1:a-1), 'k', t, result1.Var58(start:1:a-1), '-.b'); 
title('Vx'); xlabel('Time(s)'); ylabel('velocity(m/s)'); legend('Vx_ true', 'Vx_ estimated'); grid minor;

subplot(3,1,2);
plot(t, result1.Var11(start:1:a-1),'k', t, result1.Var59(start:1:a-1), '-.b');
title('Vy'); xlabel('Time(s)'); ylabel('velocity(m/s)');legend('Vy_ true', 'Vy_ estimated'); grid minor;

subplot(3,1,3);
plot(t, result1.Var12(start:1:a-1), 'k',t, result1.Var60(start:1:a-1), '-.b');
title('Vz'); xlabel('Time(s)'); ylabel('velocity(m/s)');legend('Vz_ true', 'Vz_ estimated'); grid minor;
%% RPY
figure(121); set(gcf,'units','normalized','outerposition',[0 0 1 1]);
subplot(3,1,1);
plot(t, result1.Var46(start:(finish-1)),'k',t, result1.Var94(start:(finish-1)),'-.b'); 
title('roll'); xlabel('Time(s)'); ylabel('angle(rad)');legend('roll_ true', 'roll_ estimated'); grid minor;

subplot(3,1,2);
plot(t, result1.Var47(start:(finish-1)),'k', t, result1.Var95(start:(finish-1)), '-.b')
title('Pitch'); xlabel('Time(s)'); ylabel('angle(rad)');legend('pitch_ true', 'pitch_estimated'); grid minor;

subplot(3,1,3);
plot(t, result1.Var48(start:(finish-1)),'k', t, result1.Var96(start:(finish-1)),'-.b');
title('Yaw');xlabel('Time(s)'); ylabel('angle(rad)'); legend('yaw_ true', 'yaw_estimated'); grid minor;

%% logm
figure(123); set(gcf,'units','normalized','outerposition',[0 0 1 1]);
subplot(3,1,1);
plot(t, result1.Var46(start:(a-1)),'k',t, result1.Var94(start:(a-1)),'-.b'); 
title('roll'); xlabel('Time(s)'); ylabel('angle(rad)');legend('roll_ true', 'roll_ estimated'); grid minor;

subplot(3,1,2);
plot(t, result1.Var47(start:(a-1)),'k', t, result1.Var95(start:(a-1)), '-.b')
title('Pitch'); xlabel('Time(s)'); ylabel('angle(rad)');legend('pitch_ true', 'pitch_estimated'); grid minor;

subplot(3,1,3);
plot(t, result1.Var48(start:(a-1)),'k', t, result1.Var96(start:(a-1)),'-.b');
title('Yaw');xlabel('Time(s)'); ylabel('angle(rad)'); legend('yaw_ true', 'yaw_estimated'); grid minor;

%% velocity error
% 
figure(4); set(gcf,'units','normalized','outerposition',[0 0 1 1]);

subplot(3,1,1);
% Define variable 'a' based on the desired range or condition


p = plot(t, result1.Var58(start:(finish-1)) - result1.Var10(start:(finish-1)), 'b', ...
         t, result2.Var58(start:(finish-1)) - result2.Var10(start:(finish-1)), 'r', ...
         t, result3.Var58(start:(finish-1)) - result3.Var10(start:(finish-1)), 'm', ...
         t, result4.Var58(start:(finish-1)) - result1.Var10(start:(finish-1)), ...
         t, result5.Var58(start:(finish-1)) - result1.Var10(start:(finish-1)));  
p(3).LineWidth = 1.5;
p(1).LineWidth=1.5; p(2).LineWidth=1.5; 
set(gca, 'Fontsize', 13); yline(0, '--k'); title('Vx Error', 'FontSize', 15); ylabel('Velocity(m/s)', 'FontSize', 12); grid minor; %xlabel('Time(s)', 'FontSize', 10); 
txt1 = [name1 ', RMSE: ' num2str( rms(result1.Var58(start:(a-1))-result1.Var10(start:(a-1))))];
txt2 = [name2 ', RMSE: ' num2str( rms(result2.Var58(start:(a-1))-result2.Var10(start:(a-1))))];
txt3 = [name3 ', RMSE: ' num2str( rms(result3.Var58(start:(a-1))-result3.Var10(start:(a-1))))];
txt4 = [name4 ', RMSE: ' num2str( rms(result4.Var58(start:(a-1))-result4.Var10(start:(a-1))))];
txt5 = [name5 ', RMSE: ' num2str( rms(result5.Var58(start:(a-1))-result5.Var10(start:(a-1))))];
legend(txt1, txt2, txt3, txt4, txt5, 'FontSize', 13)%, txt3, txt4, 'Reference'); 


subplot(3,1,2);
p=plot(t, result1.Var59(start:(a-1))-result1.Var11(start:(a-1)), 'b', t, result2.Var59(start:(a-1))-result2.Var11(start:(a-1)), 'r');
p=plot(t, result1.Var59(start:(a-1))-result1.Var11(start:(a-1)), 'b', t, result2.Var59(start:(a-1))-result2.Var11(start:(a-1)), 'r', t, result3.Var59(start:(a-1))-result3.Var11(start:(a-1)), 'm', t, result4.Var59(start:(a-1))-result1.Var11(start:(a-1)), t, result5.Var59(start:(a-1))-result1.Var11(start:(a-1)));
p(1).LineWidth=1.5; p(2).LineWidth=1.5; 
set(gca, 'Fontsize', 13); yline(0, '--k'); title('Vy Error', 'FontSize', 15); ylabel('Velocity(m/s)', 'FontSize', 12);grid minor; %xlabel('Time(s)', 'FontSize', 10); 
txt1 = [name1 ', RMSE: ' num2str( rms(result1.Var59(start:(a-1))-result1.Var11(start:(a-1))))];
txt2 = [name2 ', RMSE: ' num2str( rms(result2.Var59(start:(a-1))-result2.Var11(start:(a-1))))];
txt3 = [name3 ', RMSE: ' num2str( rms(result3.Var59(start:(a-1))-result3.Var11(start:(a-1))))];
txt4 = [name4 ', RMSE: ' num2str( rms(result4.Var59(start:(a-1))-result4.Var11(start:(a-1))))];
txt5 = [name5 ', RMSE: ' num2str( rms(result5.Var59(start:(a-1))-result5.Var11(start:(a-1))))];
legend(txt1, txt2,  txt3, txt4, txt5,'FontSize', 13)%,txt2, txt3, 'Reference'); 


subplot(3,1,3);
p=plot(t, result1.Var60(start:(a-1))-result1.Var12(start:(a-1)), 'b', t, result2.Var60(start:(a-1))-result2.Var12(start:(a-1)), 'r');
p=plot(t, result1.Var60(start:(a-1))-result1.Var12(start:(a-1)), 'b', t, result2.Var60(start:(a-1))-result2.Var12(start:(a-1)), 'r', t, result3.Var60(start:(a-1))-result3.Var12(start:(a-1)), 'm', t, result4.Var60(start:(a-1))-result1.Var12(start:(a-1)), t, result5.Var60(start:(a-1))-result1.Var12(start:(a-1)));
p(1).LineWidth=1.5; p(2).LineWidth=1.5; 
set(gca, 'Fontsize', 13); yline(0, '--k'); title('Vz Error', 'FontSize', 15); xlabel('Time(s)', 'FontSize', 12); ylabel('Velocity(m/s)', 'FontSize', 12);grid minor;
txt1 = [name1 ', RMSE: ' num2str( rms(result1.Var60(start:(a-1))-result1.Var12(start:(a-1))))];
txt2 = [name2 ', RMSE: ' num2str( rms(result2.Var60(start:(a-1))-result2.Var12(start:(a-1))))];
txt3 = [name3 ', RMSE: ' num2str( rms(result3.Var60(start:(a-1))-result3.Var12(start:(a-1))))];
txt4 = [name4 ', RMSE: ' num2str( rms(result4.Var60(start:(a-1))-result4.Var12(start:(a-1))))];
txt5 = [name5 ', RMSE: ' num2str( rms(result5.Var60(start:(a-1))-result5.Var12(start:(a-1))))];
%txt2 = ['Vz Error_ win=5, ' 'RMSE:' num2str( rms(result2.Var60(start:(a-1))-result1.Var12(start:(a-1))))];
%txt3 = ['Vz Error_ win=10, ' 'RMSE:' num2str( rms(result3.Var60(start:(a-1))-result1.Var12(start:(a-1))))];
legend(txt1, txt2, txt3, txt4, txt5, 'FontSize', 13)%, txt2, txt3, 'Reference'); 
% 

%% position error
figure(5); set(gcf,'units','normalized','outerposition',[0 0 1 1]);

subplot(3,1,1);
p=plot(t, result1.Var61(start:(finish-1))-result1.Var13(start:(finish-1)), 'b', t, result2.Var61(start:(finish-1))-result2.Var13(start:(finish-1)), 'r')
p=plot(t, result1.Var61(start:(finish-1))-result1.Var13(start:(finish-1)), 'b', t, result2.Var61(start:(finish-1))-result2.Var13(start:(finish-1)), 'r', t, result3.Var61(start:(finish-1))-result3.Var13(start:(finish-1)), t, result4.Var61(start:(finish-1))-result4.Var13(start:(finish-1)), t, result5.Var61(start:(finish-1))-result5.Var13(start:(finish-1)));
p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5; p(5).LineWidth=1.5; 
set(gca, 'Fontsize', 13); yline(0, '--k'); title('Px Error', 'FontSize', 15); ylabel('Position(m)', 'FontSize', 12); grid minor; %xlabel('Time(s)', 'FontSize', 10); 
txt1 = [name1 ', RMSE: ' num2str( rms(result1.Var61(start:(finish-1))-result1.Var13(start:(finish-1))))];
txt2 = [name2 ', RMSE: ' num2str( rms(result2.Var61(start:(finish-1))-result2.Var13(start:(finish-1))))];
txt3 = [name3 ', RMSE: ' num2str( rms(result3.Var61(start:(finish-1))-result3.Var13(start:(finish-1))))];
txt4 = [name4 ', RMSE: ' num2str( rms(result4.Var61(start:(finish-1))-result4.Var13(start:(finish-1))))];
txt5 = [name5 ', RMSE: ' num2str( rms(result5.Var61(start:(finish-1))-result5.Var13(start:(finish-1))))];
legend(txt1, txt2, txt3, txt4, txt5, 'FontSize', 13);%, txt3, txt4, 'Reference'); 


subplot(3,1,2);
p=plot(t, result1.Var62(start:(finish-1))-result1.Var14(start:(finish-1)), 'b', t, result2.Var62(start:(finish-1))-result2.Var14(start:(finish-1)), 'r');
p=plot(t, result1.Var62(start:(finish-1))-result1.Var14(start:(finish-1)), 'b', t, result2.Var62(start:(finish-1))-result2.Var14(start:(finish-1)), 'r', t, result3.Var62(start:(finish-1))-result3.Var14(start:(finish-1)), t, result4.Var62(start:(finish-1))-result4.Var14(start:(finish-1)), t, result5.Var62(start:(finish-1))-result5.Var14(start:(finish-1)));
p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5; p(5).LineWidth=1.5; 
set(gca, 'Fontsize', 13); yline(0, '--k'); title('Py Error', 'FontSize', 15); ylabel('Position(m)', 'FontSize', 12);grid minor; %xlabel('Time(s)', 'FontSize', 10); 
txt1 = [name1 ', RMSE: ' num2str( rms(result1.Var62(start:(finish-1))-result1.Var14(start:(finish-1))))];
txt2 = [name2 ', RMSE: ' num2str( rms(result2.Var62(start:(finish-1))-result2.Var14(start:(finish-1))))];
txt3 = [name3 ', RMSE: ' num2str( rms(result3.Var62(start:(finish-1))-result3.Var14(start:(finish-1))))];
txt4 = [name4 ', RMSE: ' num2str( rms(result4.Var62(start:(finish-1))-result4.Var14(start:(finish-1))))];
txt5 = [name5 ', RMSE: ' num2str( rms(result5.Var62(start:(finish-1))-result5.Var14(start:(finish-1))))];
legend(txt1, txt2, txt3, txt4, txt5, 'FontSize', 13);%,txt2, txt3, 'Reference'); 


subplot(3,1,3);
%p=plot(t, result1.Var63(start:(a-1))-result1.Var15(start:(a-1)), 'b', t, result2.Var63(start:(a-1))-result2.Var15(start:(a-1)), 'r');
p=plot(t, result1.Var63(start:(finish-1))-result1.Var15(start:(finish-1)), 'b', t, result2.Var63(start:(finish-1))-result2.Var15(start:(finish-1)), 'r', t, result3.Var63(start:(finish-1))-result3.Var15(start:(finish-1)), t, result4.Var63(start:(finish-1))-result4.Var15(start:(finish-1)), t, result5.Var63(start:(finish-1))-result5.Var15(start:(finish-1)));
p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5; p(5).LineWidth=1.5; 
set(gca, 'Fontsize', 13); yline(0, '--k'); title('Pz Error', 'FontSize', 15); xlabel('Time(s)', 'FontSize', 12); ylabel('Position(m)', 'FontSize', 12);grid minor;
txt1 = [name1 ', RMSE: ' num2str( rms(result1.Var63(start:(finish-1))-result1.Var15(start:(finish-1))))];
txt2 = [name2 ', RMSE: ' num2str( rms(result2.Var63(start:(finish-1))-result2.Var15(start:(finish-1))))];
txt3 = [name3 ', RMSE: ' num2str( rms(result3.Var63(start:(finish-1))-result3.Var15(start:(finish-1))))];
txt4 = [name4 ', RMSE: ' num2str( rms(result4.Var63(start:(finish-1))-result4.Var15(start:(finish-1))))];
txt5 = [name5 ', RMSE: ' num2str( rms(result5.Var63(start:(finish-1))-result5.Var15(start:(finish-1))))];
legend(txt1, txt2, txt3, txt4, txt5, 'FontSize', 13);%, txt2, txt3, 'Reference'); 


%% Rel velocity error
figure(6); set(gcf,'units','normalized','outerposition',[0 0 1 1]);

subplot(3,1,1);
%p=plot(t, v1_rel_est(start:(a-1),1) - v1_rel_true(start:(a-1),1), 'b', t, v2_rel_est(start:(a-1),1) - v2_rel_true(start:(a-1),1), 'r')
p=plot(t, v1_rel_est(start:(finish-1),1) - v1_rel_true(start:(finish-1),1), 'b', t, v2_rel_est(start:(finish-1),1) - v2_rel_true(start:(finish-1),1), 'r', t, v3_rel_est(start:(finish-1),1) - v3_rel_true(start:(finish-1),1), t, v4_rel_est(start:(finish-1),1) - v4_rel_true(start:(finish-1),1), t, v5_rel_est(start:(finish-1),1) - v5_rel_true(start:(finish-1),1));
p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5; p(5).LineWidth=1.5;
set(gca, 'Fontsize', 13); yline(0, '--k'); title('Relative Vx Error', 'FontSize', 15);  ylabel('Velocity(m/s)', 'FontSize', 12); grid minor; %xlabel('Time(s)', 'FontSize', 10);
txt1 = [name1 ', RMSE: ' num2str( rms(v1_rel_est(start:(finish-1),1) - v1_rel_true(start:(finish-1),1)))];
txt2 = [name2 ', RMSE: ' num2str( rms(v2_rel_est(start:(finish-1),1) - v2_rel_true(start:(finish-1),1)))];
txt3 = [name3 ', RMSE: ' num2str( rms(v3_rel_est(start:(finish-1),1) - v3_rel_true(start:(finish-1),1)))];
txt4 = [name4 ', RMSE: ' num2str( rms(v4_rel_est(start:(finish-1),1) - v4_rel_true(start:(finish-1),1)))];
txt5 = [name5 ', RMSE: ' num2str( rms(v5_rel_est(start:(finish-1),1) - v5_rel_true(start:(finish-1),1)))];
% txt1 = ['slip' name1 ', RMSE: ' num2str( rms(v1_rel_est(int64(slip_pt_1(1,:)./dt),1) - v1_rel_true(int64(slip_pt_1(1,:)./dt),1)))];
% txt2 = ['slip' name2 ', RMSE: ' num2str( rms(v2_rel_est(int64(slip_pt_1(1,:)./dt),1) - v2_rel_true(int64(slip_pt_1(1,:)./dt),1)))];
% txt3 = ['slip' name3 ', RMSE: ' num2str( rms(v3_rel_est(int64(slip_pt_1(1,:)./dt),1) - v3_rel_true(int64(slip_pt_1(1,:)./dt),1)))];
% txt4 = ['slip' name4 ', RMSE: ' num2str( rms(v4_rel_est(int64(slip_pt_1(1,:)./dt),1) - v4_rel_true(int64(slip_pt_1(1,:)./dt),1)))];
% txt5 = ['slip' name5 ', RMSE: ' num2str( rms(v5_rel_est(int64(slip_pt_1(1,:)./dt),1) - v5_rel_true(int64(slip_pt_1(1,:)./dt),1)))];
legend(txt1, txt2, txt3, txt4, txt5, 'FontSize', 13);%, txt3, txt4, 'Reference'); 
hold on;
b1 = bar(slip_pt_1(1,:),bar_height_plus_1(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');
b1 = bar(slip_pt_1(1,:),bar_height_minus_1(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');
% 
b2 = bar(slip_pt_2(1,:),bar_height_plus_2(1,:),1);
set(b2,'FaceAlpha',0.2, 'FaceColor',[0 0 1], 'Linestyle','none');
b2 = bar(slip_pt_2(1,:),bar_height_minus_2(1,:),1);
set(b2,'FaceAlpha',0.2, 'FaceColor',[0 0 1], 'Linestyle','none');


subplot(3,1,2);
p=plot(t, v1_rel_est(start:(finish-1),2) - v1_rel_true(start:(finish-1),2), 'b', t, v2_rel_est(start:(finish-1),2) - v2_rel_true(start:(finish-1),2), 'r');
p=plot(t, v1_rel_est(start:(finish-1),2) - v1_rel_true(start:(finish-1),2), 'b', t,v2_rel_est(start:(finish-1),2) - v2_rel_true(start:(finish-1),2), 'r', t, v3_rel_est(start:(finish-1),2) - v3_rel_true(start:(finish-1),2), t, v4_rel_est(start:(finish-1),2) - v4_rel_true(start:(finish-1),2), t, v5_rel_est(start:(finish-1),2) - v5_rel_true(start:(finish-1),2)); p(3).LineWidth=1.5;
p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5; p(5).LineWidth=1.5;
set(gca, 'Fontsize', 12); yline(0, '--k'); title('Relative Vy Error', 'FontSize', 15); ylabel('Velocity(m/s)', 'FontSize', 12);grid minor; %xlabel('Time(s)', 'FontSize', 10); 
txt1 = [name1 ', RMSE: ' num2str( rms(v1_rel_est(start:(finish-1),2) - v1_rel_true(start:(finish-1),2)))];
txt2 = [name2 ', RMSE: ' num2str( rms(v2_rel_est(start:(finish-1),2) - v2_rel_true(start:(finish-1),2)))];
txt3 = [name3 ', RMSE: ' num2str( rms(v3_rel_est(start:(finish-1),2) - v3_rel_true(start:(finish-1),2)))];
txt4 = [name4 ', RMSE: ' num2str( rms(v4_rel_est(start:(finish-1),2) - v4_rel_true(start:(finish-1),2)))];
txt5 = [name5 ', RMSE: ' num2str( rms(v5_rel_est(start:(finish-1),2) - v5_rel_true(start:(finish-1),2)))];
% txt1 = ['slip' name1 ', RMSE: ' num2str( rms(v1_rel_est(int64(slip_pt_1(1,:)./dt),2) - v1_rel_true(int64(slip_pt_1(1,:)./dt),2)))];
% txt2 = ['slip' name2 ', RMSE: ' num2str( rms(v2_rel_est(int64(slip_pt_1(1,:)./dt),2) - v2_rel_true(int64(slip_pt_1(1,:)./dt),2)))];
% txt3 = ['slip' name3 ', RMSE: ' num2str( rms(v3_rel_est(int64(slip_pt_1(1,:)./dt),2) - v3_rel_true(int64(slip_pt_1(1,:)./dt),2)))];
% txt4 = ['slip' name4 ', RMSE: ' num2str( rms(v4_rel_est(int64(slip_pt_1(1,:)./dt),2) - v4_rel_true(int64(slip_pt_1(1,:)./dt),2)))];
% txt5 = ['slip' name5 ', RMSE: ' num2str( rms(v5_rel_est(int64(slip_pt_1(1,:)./dt),2) - v5_rel_true(int64(slip_pt_1(1,:)./dt),2)))];
legend(txt1, txt2,  txt3, txt4, txt5,'FontSize', 12);%,txt2, txt3, 'Reference'); 
hold on;
b1 = bar(slip_pt_1(1,:),bar_height_plus_1(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');
b1 = bar(slip_pt_1(1,:),bar_height_minus_1(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');

% b2 = bar(slip_pt_2(1,:),bar_height_plus_2(1,:),1);
% set(b2,'FaceAlpha',0.2, 'FaceColor',[0 0 1], 'Linestyle','none');
% b2 = bar(slip_pt_2(1,:),bar_height_minus_2(1,:),1);
% set(b2,'FaceAlpha',0.2, 'FaceColor',[0 0 1], 'Linestyle','none');

subplot(3,1,3);
p=plot(t, v1_rel_est(start:(finish-1),3) - v1_rel_true(start:(finish-1),3), 'b', t, v2_rel_est(start:(finish-1),3) - v2_rel_true(start:(finish-1),3), 'r');
p=plot(t, v1_rel_est(start:(finish-1),3) - v1_rel_true(start:(finish-1),3), 'b', t, v2_rel_est(start:(finish-1),3) - v2_rel_true(start:(finish-1),3), 'r', t, v3_rel_est(start:(finish-1),3) - v3_rel_true(start:(finish-1),3), t, v4_rel_est(start:(finish-1),3) - v4_rel_true(start:(finish-1),3), t, v5_rel_est(start:(finish-1),3) - v5_rel_true(start:(finish-1),3)); p(3).LineWidth=1.5;
p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5; p(5).LineWidth=1.5;
set(gca, 'Fontsize', 13); yline(0, '--k'); title('Relative Vz Error', 'FontSize', 15); ylabel('Velocity(m/s)', 'FontSize', 12);grid minor; xlabel('Time(s)', 'FontSize', 10); 
txt1 = [name1 ', RMSE: ' num2str( rms(v1_rel_est(start:(finish-1),3) - v1_rel_true(start:(finish-1),3)))];
txt2 = [name2 ', RMSE: ' num2str( rms(v2_rel_est(start:(finish-1),3) - v2_rel_true(start:(finish-1),3)))];
txt3 = [name3 ', RMSE: ' num2str( rms(v3_rel_est(start:(finish-1),3) - v3_rel_true(start:(finish-1),3)))];
txt4 = [name4 ', RMSE: ' num2str( rms(v4_rel_est(start:(finish-1),3) - v4_rel_true(start:(finish-1),3)))];
txt5 = [name5 ', RMSE: ' num2str( rms(v5_rel_est(start:(finish-1),3) - v5_rel_true(start:(finish-1),3)))];
% txt1 = ['slip' name1 ', RMSE: ' num2str( rms(v1_rel_est(int64(slip_pt_1(1,:)./dt),3) - v1_rel_true(int64(slip_pt_1(1,:)./dt),3)))];
% txt2 = ['slip' name2 ', RMSE: ' num2str( rms(v2_rel_est(int64(slip_pt_1(1,:)./dt),3) - v2_rel_true(int64(slip_pt_1(1,:)./dt),3)))];
% txt3 = ['slip' name3 ', RMSE: ' num2str( rms(v3_rel_est(int64(slip_pt_1(1,:)./dt),3) - v3_rel_true(int64(slip_pt_1(1,:)./dt),3)))];
% txt4 = ['slip' name4 ', RMSE: ' num2str( rms(v4_rel_est(int64(slip_pt_1(1,:)./dt),3) - v4_rel_true(int64(slip_pt_1(1,:)./dt),3)))];
% txt5 = ['slip' name5 ', RMSE: ' num2str( rms(v5_rel_est(int64(slip_pt_1(1,:)./dt),3) - v5_rel_true(int64(slip_pt_1(1,:)./dt),3)))];
legend(txt1, txt2, txt3, txt4, txt5, 'FontSize', 13);%, txt2, txt3, 'Reference'); 
hold on;
b1 = bar(slip_pt_1(1,:),bar_height_plus_1(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');
b1 = bar(slip_pt_1(1,:),bar_height_minus_1(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');

% b2 = bar(slip_pt_2(1,:),bar_height_plus_2(1,:),1);
% set(b2,'FaceAlpha',0.2, 'FaceColor',[0 0 1], 'Linestyle','none');
% b2 = bar(slip_pt_2(1,:),bar_height_minus_2(1,:),1);
% set(b2,'FaceAlpha',0.2, 'FaceColor',[0 0 1], 'Linestyle','none');
%% RPY error
figure(31); set(gcf,'units','normalized','outerposition',[0 0 1 1]);
title('rpy_error');

subplot(3,1,1);
%p=plot(t, result1.Var94(start:(a-1))-result1.Var46(start:(a-1)), 'b', t, result2.Var94(start:(a-1))-result1.Var46(start:(a-1)), 'r')
%p=plot(t, result1.Var94(start:(a-1))-result1.Var46(start:(a-1)), 'b', t, result2.Var94(start:(a-1))-result1.Var46(start:(a-1)), 'r', t, result3.Var94(start:(a-1))-result1.Var46(start:(a-1))); p(3).LineWidth=1.5;
%p=plot(t, rot_error1(start:(a-1),3), 'b', t, rot_error2(start:(a-1),3), 'r')
p=plot(t, result1.Var94(start:(finish-1))-result1.Var46(start:(finish-1)), 'b', t, result2.Var94(start:(finish-1))-result1.Var46(start:(finish-1)), 'r', t, result3.Var94(start:(finish-1))-result1.Var46(start:(finish-1)), t, result4.Var94(start:(finish-1))-result1.Var46(start:(finish-1)), t, result5.Var94(start:(finish-1))-result1.Var46(start:(finish-1)) ); p(3).LineWidth=1.5;
p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5; p(5).LineWidth=1.5;
set(gca, 'Fontsize', 13); yline(0, '--k'); title(' rpy Error Roll', 'FontSize', 15); ylabel('Angle(rad)', 'FontSize', 12); grid minor; %xlabel('Time(s)', 'FontSize', 10); 
txt1 = [name1 ', RMSE: ' num2str( rms(result1.Var94(start:(finish-1))-result1.Var46(start:(finish-1))))];
txt2 = [name2 ', RMSE: ' num2str( rms(result2.Var94(start:(finish-1))-result2.Var46(start:(finish-1))))];
txt3 = [name3 ', RMSE: ' num2str( rms(result3.Var94(start:(finish-1))-result3.Var46(start:(finish-1))))];
txt4 = [name4 ', RMSE: ' num2str( rms(result4.Var94(start:(finish-1))-result4.Var46(start:(finish-1))))];
txt5 = [name5 ', RMSE: ' num2str( rms(result5.Var94(start:(finish-1))-result5.Var46(start:(finish-1))))];
%txt2 = ['Roll Error_ win=5, ' 'RMSE:' num2str( rms(result2.Var94(start:(a-1))-result1.Var46(start:(a-1))))];
%txt3 = ['Roll Error_ win=10, ' 'RMSE:' num2str( rms(result3.Var94(start:(a-1))-result1.Var46(start:(a-1))))];
legend(txt1, txt2,txt3, txt4, txt5, 'FontSize', 13);%, txt2, txt3, 'Reference'); 
hold on;
b1 = bar(slip_pt_1(1,:),bar_height_plus_1(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');
b1 = bar(slip_pt_1(1,:),bar_height_minus_1(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');

subplot(3,1,2);
% p=plot(t, result1.Var95(start:(a-1))-result1.Var47(start:(a-1)), 'b', t, result2.Var95(start:(a-1))-result1.Var47(start:(a-1)), 'r');
% p=plot(t, result1.Var95(start:(a-1))-result1.Var47(start:(a-1)), 'b', t, result2.Var95(start:(a-1))-result1.Var47(start:(a-1)), 'r', t, result3.Var95(start:(a-1))-result1.Var47(start:(a-1))); p(3).LineWidth=1.5;
p=plot(t, rot_error1(start:(finish-1),2), 'b', t, rot_error2(start:(finish-1),2), 'r')
p=plot(t, result1.Var95(start:(finish-1))-result1.Var47(start:(finish-1)), 'b', t, result2.Var95(start:(finish-1))-result2.Var47(start:(finish-1)), 'r', t, result3.Var95(start:(finish-1))-result3.Var47(start:(finish-1)), t, result4.Var95(start:(finish-1))-result4.Var47(start:(finish-1)), t, result5.Var95(start:(finish-1))-result5.Var47(start:(finish-1)) ); p(3).LineWidth=1.5;
p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5; p(5).LineWidth=1.5;
set(gca, 'Fontsize', 13); yline(0, '--k'); title('rpy Error Pitch', 'FontSize', 15); ylabel('Angle(rad)', 'FontSize', 12); grid minor; %xlabel('Time(s)', 'FontSize', 10); 
txt1 = [name1 ', RMSE: ' num2str( rms(result1.Var95(start:(finish-1))-result1.Var47(start:(finish-1))))];
txt2 = [name2 ', RMSE: ' num2str( rms(result2.Var95(start:(finish-1))-result2.Var47(start:(finish-1))))];
txt3 = [name3 ', RMSE: ' num2str( rms(result3.Var95(start:(finish-1))-result3.Var47(start:(finish-1))))];
txt4 = [name4 ', RMSE: ' num2str( rms(result4.Var95(start:(finish-1))-result4.Var47(start:(finish-1))))];
txt5 = [name5 ', RMSE: ' num2str( rms(result5.Var95(start:(finish-1))-result5.Var47(start:(finish-1))))];
%txt2 = ['Pitch Error_ win=5, ' 'RMSE:' num2str( rms(result2.Var95(start:(a-1))-result1.Var47(start:(a-1))))];
%txt3 = ['Pitch Error_ win=10, ' 'RMSE:' num2str( rms(result3.Var95(start:(a-1))-result1.Var47(start:(a-1))))];
legend(txt1, txt2,txt3, txt4, txt5, 'FontSize', 13);%, txt2, txt3, 'Reference'); 
hold on;
b1 = bar(slip_pt_1(1,:),bar_height_plus_1(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');
b1 = bar(slip_pt_1(1,:),bar_height_minus_1(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');


subplot(3,1,3);
% p=plot(t, result1.Var96(start:(a-1))-result1.Var48(start:(a-1)), 'b', t, result2.Var96(start:(a-1))-result1.Var48(start:(a-1)), 'r');
% p=plot(t, result1.Var96(start:(a-1))-result1.Var48(start:(a-1)), 'b', t, result2.Var96(start:(a-1))-result1.Var48(start:(a-1)), 'r', t, result3.Var96(start:(a-1))-result1.Var48(start:(a-1)));  p(3).LineWidth=1.5; 
p=plot(t, rot_error1(start:(finish-1),1), 'b', t, rot_error2(start:(finish-1),1), 'r')
p=plot(t, result1.Var96(start:(finish-1))-result1.Var48(start:(finish-1)), 'b', t, result2.Var96(start:(finish-1))-result2.Var48(start:(finish-1)), 'r', t, result3.Var96(start:(finish-1))-result3.Var48(start:(finish-1)), t, result4.Var96(start:(finish-1))-result4.Var48(start:(finish-1)), t, result5.Var96(start:(finish-1))-result5.Var48(start:(finish-1)) ); p(3).LineWidth=1.5;
p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5; p(5).LineWidth=1.5;
set(gca, 'Fontsize', 13); yline(0, '--k'); title('rpy Error Yaw', 'FontSize', 15); ylabel('Angle(rad)', 'FontSize', 12);grid minor; xlabel('Time(s)', 'FontSize', 10);
txt1 = [name1 ', RMSE: ' num2str( rms(result1.Var96(start:(finish-1))-result1.Var48(start:(finish-1))))];
txt2 = [name2 ', RMSE: ' num2str( rms(result2.Var96(start:(finish-1))-result2.Var48(start:(finish-1))))];
txt3 = [name3 ', RMSE: ' num2str( rms(result3.Var96(start:(finish-1))-result3.Var48(start:(finish-1))))];
txt4 = [name4 ', RMSE: ' num2str( rms(result4.Var96(start:(finish-1))-result4.Var48(start:(finish-1))))];
txt5 = [name5 ', RMSE: ' num2str( rms(result5.Var96(start:(finish-1))-result5.Var48(start:(finish-1))))];
%txt2 = ['Yaw Error_ win=5, ' 'RMSE:' num2str( rms(result2.Var96(start:(a-1))-result1.Var48(start:(a-1))))];
%txt3 = ['Yaw Error_ win=10, ' 'RMSE:' num2str( rms(result3.Var96(start:(a-1))-result1.Var48(start:(a-1))))];
legend(txt1, txt2, txt3, txt4, txt5, 'FontSize', 13);%, txt2, txt3, 'Reference'); 
hold on;
b1 = bar(slip_pt_1(1,:),bar_height_plus_1(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');
b1 = bar(slip_pt_1(1,:),bar_height_minus_1(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');


%% Rot error
figure(32); set(gcf,'units','normalized','outerposition',[0 0 1 1]);
title('Rot_error');

subplot(3,1,1);
%p=plot(t, result1.Var94(start:(a-1))-result1.Var46(start:(a-1)), 'b', t, result2.Var94(start:(a-1))-result1.Var46(start:(a-1)), 'r')
%p=plot(t, result1.Var94(start:(a-1))-result1.Var46(start:(a-1)), 'b', t, result2.Var94(start:(a-1))-result1.Var46(start:1:a-1), 'r', t, result3.Var94(start:1:a-1)-result1.Var46(start:1:a-1)); p(3).LineWidth=1.5;
p=plot(t, rot_error1(start:1:a-1,1), 'b', t, rot_error2(start:1:a-1,1), 'r')
p=plot(t, rot_error1(start:1:a-1,1), 'b', t, rot_error2(start:1:a-1,1), 'r', t, rot_error3(start:1:a-1,1), t, rot_error4(start:1:a-1,1), t, rot_error5(start:1:a-1,1) ); p(3).LineWidth=1.5;
p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5; p(5).LineWidth=1.5;
set(gca, 'Fontsize', 13); yline(0, '--k'); title(' R^TR Error Roll', 'FontSize', 15); ylabel('Angle(rad)', 'FontSize', 12); grid minor; %xlabel('Time(s)', 'FontSize', 10); 
txt1 = [name1 ', RMSE: ' num2str( rms(rot_error1(start:1:a-1,1)))];
txt2 = [name2 ', RMSE: ' num2str( rms(rot_error2(start:1:a-1,1)))];
txt3 = [name3 ', RMSE: ' num2str( rms(rot_error3(start:1:a-1,1)))];
txt4 = [name4 ', RMSE: ' num2str( rms(rot_error4(start:1:a-1,1)))];
txt5 = [name5 ', RMSE: ' num2str( rms(rot_error5(start:1:a-1,1)))];
%txt2 = ['Roll Error_ win=5, ' 'RMSE:' num2str( rms(result2.Var94(start:1:a-1)-result1.Var46(start:1:a-1)))];
%txt3 = ['Roll Error_ win=10, ' 'RMSE:' num2str( rms(result3.Var94(start:1:a-1)-result1.Var46(start:1:a-1)))];
legend(txt1, txt2,txt3, txt4, txt5, 'FontSize', 13);%, txt2, txt3, 'Reference'); 
hold on;
b1 = bar(slip_pt(1,:),bar_height_plus(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');
b1 = bar(slip_pt(1,:),bar_height_minus(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');

subplot(3,1,2);
% p=plot(t, result1.Var95(start:1:a-1)-result1.Var47(start:1:a-1), 'b', t, result2.Var95(start:1:a-1)-result1.Var47(start:1:a-1), 'r');
% p=plot(t, result1.Var95(start:1:a-1)-result1.Var47(start:1:a-1), 'b', t, result2.Var95(start:1:a-1)-result1.Var47(start:1:a-1), 'r', t, result3.Var95(start:1:a-1)-result1.Var47(start:1:a-1)); p(3).LineWidth=1.5;
p=plot(t, rot_error1(start:1:a-1,2), 'b', t, rot_error2(start:1:a-1,2), 'r')
p=plot(t, rot_error1(start:1:a-1,2), 'b', t, rot_error2(start:1:a-1,2), 'r', t, rot_error3(start:1:a-1,2), t, rot_error4(start:1:a-1,2), t, rot_error5(start:1:a-1,2) ); p(3).LineWidth=1.5;
p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5; p(5).LineWidth=1.5;
set(gca, 'Fontsize', 13); yline(0, '--k'); title('R^TR Error Pitch', 'FontSize', 15); ylabel('Angle(rad)', 'FontSize', 12); grid minor; %xlabel('Time(s)', 'FontSize', 10); 
txt1 = [name1 ', RMSE: ' num2str( rms(rot_error1(start:1:a-1,2)))];
txt2 = [name2 ', RMSE: ' num2str( rms(rot_error2(start:1:a-1,2)))];
txt3 = [name3 ', RMSE: ' num2str( rms(rot_error3(start:1:a-1,2)))];
txt4 = [name4 ', RMSE: ' num2str( rms(rot_error4(start:1:a-1,2)))];
txt5 = [name5 ', RMSE: ' num2str( rms(rot_error5(start:1:a-1,2)))];
%txt2 = ['Pitch Error_ win=5, ' 'RMSE:' num2str( rms(result2.Var95(start:1:a-1)-result1.Var47(start:1:a-1)))];
%txt3 = ['Pitch Error_ win=10, ' 'RMSE:' num2str( rms(result3.Var95(start:1:a-1)-result1.Var47(start:1:a-1)))];
legend(txt1, txt2,txt3, txt4, txt5, 'FontSize', 13);%, txt2, txt3, 'Reference'); 
hold on;
b1 = bar(slip_pt(1,:),bar_height_plus(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');
b1 = bar(slip_pt(1,:),bar_height_minus(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');


subplot(3,1,3);
% p=plot(t, result1.Var96(start:1:a-1)-result1.Var48(start:1:a-1), 'b', t, result2.Var96(start:1:a-1)-result1.Var48(start:1:a-1), 'r');
% p=plot(t, result1.Var96(start:1:a-1)-result1.Var48(start:1:a-1), 'b', t, result2.Var96(start:1:a-1)-result1.Var48(start:1:a-1), 'r', t, result3.Var96(start:1:a-1)-result1.Var48(start:1:a-1));  p(3).LineWidth=1.5; 
p=plot(t, rot_error1(start:1:a-1,1), 'b', t, rot_error2(start:1:a-1,1), 'r')
p=plot(t, rot_error1(start:1:a-1,3), 'b', t, rot_error2(start:1:a-1,3), 'r', t, rot_error3(start:1:a-1,3), t, rot_error4(start:1:a-1,3), t, rot_error5(start:1:a-1,3) ); p(3).LineWidth=1.5;
p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5; p(5).LineWidth=1.5;
set(gca, 'Fontsize', 13); yline(0, '--k'); title('R^TR Error Yaw', 'FontSize', 15); ylabel('Angle(rad)', 'FontSize', 12);grid minor; xlabel('Time(s)', 'FontSize', 10);
txt1 = [name1 ', RMSE: ' num2str( rms(rot_error1(start:1:a-1,3)))];
txt2 = [name2 ', RMSE: ' num2str( rms(rot_error2(start:1:a-1,3)))];
txt3 = [name3 ', RMSE: ' num2str( rms(rot_error3(start:1:a-1,3)))];
txt4 = [name4 ', RMSE: ' num2str( rms(rot_error4(start:1:a-1,3)))];
txt5 = [name5 ', RMSE: ' num2str( rms(rot_error5(start:1:a-1,3)))];
%txt2 = ['Yaw Error_ win=5, ' 'RMSE:' num2str( rms(result2.Var96(start:1:a-1)-result1.Var48(start:1:a-1)))];
%txt3 = ['Yaw Error_ win=10, ' 'RMSE:' num2str( rms(result3.Var96(start:1:a-1)-result1.Var48(start:1:a-1)))];
legend(txt1, txt2, txt3, txt4, txt5, 'FontSize', 13);%, txt2, txt3, 'Reference'); 
hold on;
b1 = bar(slip_pt(1,:),bar_height_plus(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');
b1 = bar(slip_pt(1,:),bar_height_minus(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');

%% logm error
figure(33); set(gcf,'units','normalized','outerposition',[0 0 1 1]);
title('logm_error');

subplot(3,1,1);
%p=plot(t, result1.Var94(start:1:a-1)-result1.Var46(start:1:a-1), 'b', t, result2.Var94(start:1:a-1)-result1.Var46(start:1:a-1), 'r')
%p=plot(t, result1.Var94(start:1:a-1)-result1.Var46(start:1:a-1), 'b', t, result2.Var94(start:1:a-1)-result1.Var46(start:1:a-1), 'r', t, result3.Var94(start:1:a-1)-result1.Var46(start:1:a-1)); p(3).LineWidth=1.5;
p=plot(t, phi_error1(start:1:finish-1,1), 'b', t, phi_error2(start:1:finish-1,3), 'r')
p=plot(t, phi_error1(start:1:finish-1,1), 'b', t, phi_error2(start:1:finish-1,1), 'r', t, phi_error3(start:1:finish-1,1), t, phi_error4(start:1:finish-1,1), t, phi_error5(start:1:finish-1,1) ); p(3).LineWidth=1.5;
p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5; p(5).LineWidth=1.5;
set(gca, 'Fontsize', 13); yline(0, '--k'); title(' logm Error Roll', 'FontSize', 15); ylabel('Angle(rad)', 'FontSize', 12); grid minor; %xlabel('Time(s)', 'FontSize', 10); 
txt1 = [name1 ', RMSE: ' num2str( rms(phi_error1(start:1:finish-1,1)))];
txt2 = [name2 ', RMSE: ' num2str( rms(phi_error2(start:1:finish-1,1)))];
txt3 = [name3 ', RMSE: ' num2str( rms(phi_error3(start:1:finish-1,1)))];
txt4 = [name4 ', RMSE: ' num2str( rms(phi_error4(start:1:finish-1,1)))];
txt5 = [name5 ', RMSE: ' num2str( rms(phi_error5(start:1:finish-1,1)))];
%txt2 = ['Roll Error_ win=5, ' 'RMSE:' num2str( rms(result2.Var94(start:1:a-1)-result1.Var46(start:1:a-1)))];
%txt3 = ['Roll Error_ win=10, ' 'RMSE:' num2str( rms(result3.Var94(start:1:a-1)-result1.Var46(start:1:a-1)))];
legend(txt1, txt2,txt3, txt4, txt5, 'FontSize', 13);%, txt2, txt3, 'Reference'); 
hold on;
b1 = bar(slip_pt_1(1,:),bar_height_plus_1(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');
b1 = bar(slip_pt_1(1,:),bar_height_minus_1(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');

subplot(3,1,2);
% p=plot(t, result1.Var95(start:1:a-1)-result1.Var47(start:1:a-1), 'b', t, result2.Var95(start:1:a-1)-result1.Var47(start:1:a-1), 'r');
% p=plot(t, result1.Var95(start:1:a-1)-result1.Var47(start:1:a-1), 'b', t, result2.Var95(start:1:a-1)-result1.Var47(start:1:a-1), 'r', t, result3.Var95(start:1:a-1)-result1.Var47(start:1:a-1)); p(3).LineWidth=1.5;
p=plot(t, phi_error1(start:1:finish-1,2), 'b', t, phi_error2(start:1:finish-1,2), 'r')
p=plot(t, phi_error1(start:1:finish-1,2), 'b', t, phi_error2(start:1:finish-1,2), 'r', t, phi_error3(start:1:finish-1,2), t, phi_error4(start:1:finish-1,2), t, phi_error5(start:1:finish-1,2) ); p(3).LineWidth=1.5;
p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5; p(5).LineWidth=1.5;
set(gca, 'Fontsize', 13); yline(0, '--k'); title('logm Error Pitch', 'FontSize', 15); ylabel('Angle(rad)', 'FontSize', 12); grid minor; %xlabel('Time(s)', 'FontSize', 10); 
txt1 = [name1 ', RMSE: ' num2str( rms(phi_error1(start:1:finish-1,2)))];
txt2 = [name2 ', RMSE: ' num2str( rms(phi_error2(start:1:finish-1,2)))];
txt3 = [name3 ', RMSE: ' num2str( rms(phi_error3(start:1:finish-1,2)))];
txt4 = [name4 ', RMSE: ' num2str( rms(phi_error4(start:1:finish-1,2)))];
txt5 = [name5 ', RMSE: ' num2str( rms(phi_error5(start:1:finish-1,2)))];
%txt2 = ['Pitch Error_ win=5, ' 'RMSE:' num2str( rms(result2.Var95(start:1:a-1)-result1.Var47(start:1:a-1)))];
%txt3 = ['Pitch Error_ win=10, ' 'RMSE:' num2str( rms(result3.Var95(start:1:a-1)-result1.Var47(start:1:a-1)))];
legend(txt1, txt2,txt3, txt4, txt5, 'FontSize', 13);%, txt2, txt3, 'Reference'); 
hold on;
b1 = bar(slip_pt_1(1,:),bar_height_plus_1(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');
b1 = bar(slip_pt_1(1,:),bar_height_minus_1(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');


subplot(3,1,3);
% p=plot(t, result1.Var96(start:1:a-1)-result1.Var48(start:1:a-1), 'b', t, result2.Var96(start:1:a-1)-result1.Var48(start:1:a-1), 'r');
% p=plot(t, result1.Var96(start:1:a-1)-result1.Var48(start:1:a-1), 'b', t, result2.Var96(start:1:a-1)-result1.Var48(start:1:a-1), 'r', t, result3.Var96(start:1:a-1)-result1.Var48(start:1:a-1));  p(3).LineWidth=1.5; 
p=plot(t, phi_error1(start:1:finish-1,3), 'b', t, phi_error2(start:1:finish-1,1), 'r')
p=plot(t, phi_error1(start:1:finish-1,3), 'b', t, phi_error2(start:1:finish-1,3), 'r', t, phi_error3(start:1:finish-1,3), t, phi_error4(start:1:finish-1,3), t, phi_error5(start:1:finish-1,3) ); p(3).LineWidth=1.5;
p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5; p(5).LineWidth=1.5;
set(gca, 'Fontsize', 13); yline(0, '--k'); title('logm Error Yaw', 'FontSize', 15); ylabel('Angle(rad)', 'FontSize', 12);grid minor; xlabel('Time(s)', 'FontSize', 10);
txt1 = [name1 ', RMSE: ' num2str( rms(phi_error1(start:1:finish-1,3)))];
txt2 = [name2 ', RMSE: ' num2str( rms(phi_error2(start:1:finish-1,3)))];
txt3 = [name3 ', RMSE: ' num2str( rms(phi_error3(start:1:finish-1,3)))];
txt4 = [name4 ', RMSE: ' num2str( rms(phi_error4(start:1:finish-1,3)))];
txt5 = [name5 ', RMSE: ' num2str( rms(phi_error5(start:1:finish-1,3)))];
%txt2 = ['Yaw Error_ win=5, ' 'RMSE:' num2str( rms(result2.Var96(start:1:a-1)-result1.Var48(start:1:a-1)))];
%txt3 = ['Yaw Error_ win=10, ' 'RMSE:' num2str( rms(result3.Var96(start:1:a-1)-result1.Var48(start:1:a-1)))];
legend(txt1, txt2, txt3, txt4, txt5, 'FontSize', 13);%, txt2, txt3, 'Reference'); 
hold on;
b1 = bar(slip_pt_1(1,:),bar_height_plus_1(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');
b1 = bar(slip_pt_1(1,:),bar_height_minus_1(1,:),1);
set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');


% %% H rpy error
% figure(34); set(gcf,'units','normalized','outerposition',[0 0 1 1]);
% title('logm_error');
% 
% subplot(3,1,1);
% %p=plot(t, result1.Var94(start:1:a-1)-result1.Var46(start:1:a-1), 'b', t, result2.Var94(start:1:a-1)-result1.Var46(start:1:a-1), 'r')
% %p=plot(t, result1.Var94(start:1:a-1)-result1.Var46(start:1:a-1), 'b', t, result2.Var94(start:1:a-1)-result1.Var46(start:1:a-1), 'r', t, result3.Var94(start:1:a-1)-result1.Var46(start:1:a-1)); p(3).LineWidth=1.5;
% %p=plot(t, phi_error1(start:1:a-1,1), 'b', t, phi_error2(start:1:a-1,3), 'r')
% p=plot(t, Hrpy_error1(start:1:a-1,1), 'b', t, Hrpy_error2(start:1:a-1,1), 'r', t, Hrpy_error3(start:1:a-1,1), t, Hrpy_error4(start:1:a-1,1), t, Hrpy_error5(start:1:a-1,1) ); p(3).LineWidth=1.5;
% p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5; p(5).LineWidth=1.5;
% set(gca, 'Fontsize', 13); yline(0, '--k'); title(' H rpy Error Roll', 'FontSize', 15); ylabel('Angle(rad)', 'FontSize', 12); grid minor; %xlabel('Time(s)', 'FontSize', 10); 
% txt1 = [name1 ', RMSE: ' num2str( rms(Hrpy_error1(start:1:a-1,1)))];
% txt2 = [name2 ', RMSE: ' num2str( rms(Hrpy_error2(start:1:a-1,1)))];
% txt3 = [name3 ', RMSE: ' num2str( rms(Hrpy_error3(start:1:a-1,1)))];
% txt4 = [name4 ', RMSE: ' num2str( rms(Hrpy_error4(start:1:a-1,1)))];
% txt5 = [name5 ', RMSE: ' num2str( rms(Hrpy_error5(start:1:a-1,1)))];
% %txt2 = ['Roll Error_ win=5, ' 'RMSE:' num2str( rms(result2.Var94(start:1:a-1)-result1.Var46(start:1:a-1)))];
% %txt3 = ['Roll Error_ win=10, ' 'RMSE:' num2str( rms(result3.Var94(start:1:a-1)-result1.Var46(start:1:a-1)))];
% legend(txt1, txt2,txt3, txt4, txt5, 'FontSize', 13);%, txt2, txt3, 'Reference'); 
% hold on;
% b1 = bar(slip_pt(1,:),bar_height_plus(1,:),1);
% set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');
% b1 = bar(slip_pt(1,:),bar_height_minus(1,:),1);
% set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');
% 
% subplot(3,1,2);
% % p=plot(t, result1.Var95(start:1:a-1)-result1.Var47(start:1:a-1), 'b', t, result2.Var95(start:1:a-1)-result1.Var47(start:1:a-1), 'r');
% % p=plot(t, result1.Var95(start:1:a-1)-result1.Var47(start:1:a-1), 'b', t, result2.Var95(start:1:a-1)-result1.Var47(start:1:a-1), 'r', t, result3.Var95(start:1:a-1)-result1.Var47(start:1:a-1)); p(3).LineWidth=1.5;
% %p=plot(t, phi_error1(start:1:a-1,2), 'b', t, phi_error2(start:1:a-1,2), 'r')
% p=plot(t, Hrpy_error1(start:1:a-1,2), 'b', t, Hrpy_error2(start:1:a-1,2), 'r', t, Hrpy_error3(start:1:a-1,2), t, Hrpy_error4(start:1:a-1,2), t, Hrpy_error5(start:1:a-1,2) ); p(3).LineWidth=1.5;
% p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5; p(5).LineWidth=1.5;
% set(gca, 'Fontsize', 13); yline(0, '--k'); title('H rpy Error Pitch', 'FontSize', 15); ylabel('Angle(rad)', 'FontSize', 12); grid minor; %xlabel('Time(s)', 'FontSize', 10); 
% txt1 = [name1 ', RMSE: ' num2str( rms(Hrpy_error1(start:1:a-1,2)))];
% txt2 = [name2 ', RMSE: ' num2str( rms(Hrpy_error2(start:1:a-1,2)))];
% txt3 = [name3 ', RMSE: ' num2str( rms(Hrpy_error3(start:1:a-1,2)))];
% txt4 = [name4 ', RMSE: ' num2str( rms(Hrpy_error4(start:1:a-1,2)))];
% txt5 = [name5 ', RMSE: ' num2str( rms(Hrpy_error5(start:1:a-1,2)))];
% %txt2 = ['Pitch Error_ win=5, ' 'RMSE:' num2str( rms(result2.Var95(start:1:a-1)-result1.Var47(start:1:a-1)))];
% %txt3 = ['Pitch Error_ win=10, ' 'RMSE:' num2str( rms(result3.Var95(start:1:a-1)-result1.Var47(start:1:a-1)))];
% legend(txt1, txt2,txt3, txt4, txt5, 'FontSize', 13);%, txt2, txt3, 'Reference'); 
% hold on;
% b1 = bar(slip_pt(1,:),bar_height_plus(1,:),1);
% set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');
% b1 = bar(slip_pt(1,:),bar_height_minus(1,:),1);
% set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');
% 
% 
% subplot(3,1,3);
% % p=plot(t, result1.Var96(start:1:a-1)-result1.Var48(start:1:a-1), 'b', t, result2.Var96(start:1:a-1)-result1.Var48(start:1:a-1), 'r');
% % p=plot(t, result1.Var96(start:1:a-1)-result1.Var48(start:1:a-1), 'b', t, result2.Var96(start:1:a-1)-result1.Var48(start:1:a-1), 'r', t, result3.Var96(start:1:a-1)-result1.Var48(start:1:a-1));  p(3).LineWidth=1.5; 
% %p=plot(t, phi_error1(start:1:a-1,3), 'b', t, phi_error2(start:1:a-1,3), 'r')
% p=plot(t, Hrpy_error1(start:1:a-1,3), 'b', t, Hrpy_error2(start:1:a-1,3), 'r', t, Hrpy_error3(start:1:a-1,3), t, Hrpy_error4(start:1:a-1,3), t, Hrpy_error5(start:1:a-1,3) ); p(3).LineWidth=1.5;
% p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5; p(5).LineWidth=1.5;
% set(gca, 'Fontsize', 13); yline(0, '--k'); title('H rpy Error Yaw', 'FontSize', 15); ylabel('Angle(rad)', 'FontSize', 12);grid minor; xlabel('Time(s)', 'FontSize', 10);
% txt1 = [name1 ', RMSE: ' num2str( rms(Hrpy_error1(start:1:a-1,3)))];
% txt2 = [name2 ', RMSE: ' num2str( rms(Hrpy_error2(start:1:a-1,3)))];
% txt3 = [name3 ', RMSE: ' num2str( rms(Hrpy_error3(start:1:a-1,3)))];
% txt4 = [name4 ', RMSE: ' num2str( rms(Hrpy_error4(start:1:a-1,3)))];
% txt5 = [name5 ', RMSE: ' num2str( rms(Hrpy_error5(start:1:a-1,3)))];
% %txt2 = ['Yaw Error_ win=5, ' 'RMSE:' num2str( rms(result2.Var96(start:1:a-1)-result1.Var48(start:1:a-1)))];
% %txt3 = ['Yaw Error_ win=10, ' 'RMSE:' num2str( rms(result3.Var96(start:1:a-1)-result1.Var48(start:1:a-1)))];
% legend(txt1, txt2, txt3, txt4, txt5, 'FontSize', 13);%, txt2, txt3, 'Reference'); 
% hold on;
% b1 = bar(slip_pt(1,:),bar_height_plus(1,:),1);
% set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');
% b1 = bar(slip_pt(1,:),bar_height_minus(1,:),1);
% set(b1,'FaceAlpha',0.2, 'FaceColor',[1 0 0], 'Linestyle','none');



% %% iterations
% figure(8);
% set(gcf,'units','normalized','outerposition',[0 0 1 1]);
% 
% %Y=[result1.Var97(start:1:finish-1), result2.Var97(start:1:finish-1)]
% %stem(t, Y);
% p=plot(t, result1.Var97(start:1:finish-1), 'b', t, result2.Var97(start:1:finish-1), 'r', t, result3.Var97(start:1:finish-1),t, result4.Var97(start:1:finish-1))%, t, result4.Var97(start:1:finish-1), 'r');
% p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5;
% set(gca, 'Fontsize', 12); title('Iterations', 'FontSize', 15); xlabel('Time(s)', 'FontSize', 10); ylabel('Iteration No.', 'FontSize', 10);grid minor;
% txt1 = [name1 ', Mean Iteration No. = ' num2str( mean(result1.Var97(start:1:finish-1)))];
% txt2 = [name2 ', Mean Iteration No. = ' num2str( mean(result2.Var97(start:1:finish-1)))];
% txt3 = [name3 ', Mean Iteration No. = ' num2str( mean(result3.Var97(start:1:finish-1)))];
% txt4 = [name4 ', Mean Iteration No. = ' num2str( mean(result4.Var97(start:1:finish-1)))];
% legend(txt1, txt2, txt3, txt4, 'FontSize', 12);

% %% backpps
% figure(9);
% set(gcf,'units','normalized','outerposition',[0 0 1 1]);
% 
% %Y=[result1.Var97(start:1:finish-1), result2.Var97(start:1:finish-1)]
% %stem(t, Y);
% p=plot(t, result1.Var98(start:1:finish-1), 'b', t, result2.Var98(start:1:finish-1), 'r', t, result3.Var98(start:1:finish-1),t, result4.Var98(start:1:finish-1))%, t, result4.Var97(start:1:finish-1), 'r');
% p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5;
% set(gca, 'Fontsize', 12); title('Iterations', 'FontSize', 15); xlabel('Time(s)', 'FontSize', 10); ylabel('Iteration No.', 'FontSize', 10);grid minor;
% txt1 = [name1 ', Mean Iteration No. = ' num2str( mean(result1.Var98(start:1:finish-1)))];
% txt2 = [name2 ', Mean Iteration No. = ' num2str( mean(result2.Var98(start:1:finish-1)))];
% txt3 = [name3 ', Mean Iteration No. = ' num2str( mean(result3.Var98(start:1:finish-1)))];
% txt4 = [name4 ', Mean Iteration No. = ' num2str( mean(result4.Var98(start:1:finish-1)))];
% legend(txt1, txt2, txt3, txt4, 'FontSize', 12);
% % 
% % % 
% %% Cost
% figure(10);
% set(gcf,'units','normalized','outerposition',[0 0 1 1]);
% 
% %Y=[result1.Var97(start:1:finish-1), result2.Var97(start:1:finish-1)]
% %stem(t, Y);
% p=plot(t, result1.Var100(start:1:finish-1), 'b', t, result2.Var100(start:1:finish-1), 'r', t, result3.Var100(start:1:finish-1),t, result4.Var100(start:1:finish-1))%, t, result4.Var97(start:1:finish-1), 'r');
% p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5;
% set(gca, 'Fontsize', 12); title('Cost', 'FontSize', 15); xlabel('Time(s)', 'FontSize', 10); ylabel('Iteration No.', 'FontSize', 10);grid minor;
% txt1 = [name1 ', Mean Iteration No. = ' num2str( mean(result1.Var100(start:1:finish-1)))];
% txt2 = [name2 ', Mean Iteration No. = ' num2str( mean(result2.Var100(start:1:finish-1)))];
% txt3 = [name3 ', Mean Iteration No. = ' num2str( mean(result3.Var100(start:1:finish-1)))];
% txt4 = [name4 ', Mean Iteration No. = ' num2str( mean(result4.Var100(start:1:finish-1)))];
% legend(txt1, txt2, txt3, txt4, 'FontSize', 12);
% % % 
% % 
% % % index
% % figure(10);
% % set(gcf,'units','normalized','outerposition',[0 0 1 1]);
% % 
% % %Y=[result1.Var97(start:1:a-1), result2.Var97(start:1:a-1)]
% % %stem(t, Y);
% % 
% % p=plot(t, result1.Var99(start:1:a-1), 'b', t, result2.Var99(start:1:a-1), 'r', t, result3.Var99(start:1:a-1),t, result4.Var99(start:1:a-1))%, t, result4.Var97(start:1:a-1), 'r');
% % p(1).LineWidth=1.5; p(2).LineWidth=1.5; p(3).LineWidth=1.5; p(4).LineWidth=1.5;
% % set(gca, 'Fontsize', 12); title('Operating Time', 'FontSize', 15); xlabel('Time(s)', 'FontSize', 10); ylabel('Operating Time(s)', 'FontSize', 10);grid minor;
% % txt1 = [name1 ', Mean Operating Time = ' num2str( mean(result1.Var99(start:1:a-1)))];
% % txt2 = [name2 ', Mean Operating Time = ' num2str( mean(result2.Var99(start:1:a-1)))];
% % txt2 = [name3 ', Mean Operating Time = ' num2str( mean(result3.Var99(start:1:a-1)))];
% % legend(txt1, txt2, txt3, 'FontSize', 12);


%% dv

% d1_a(1)=0; d2_a(1)=0;d3_a(1)=0; d4_a(1)=0; 
% for k=2:length(d1_v)
%     d1_a(k) = d1_v(k,1)-d1_v(k-1,1);
%     d2_a(k) = d2_v(k,1)-d2_v(k-1,1);
%     d3_a(k) = d3_v(k,1)-d3_v(k-1,1);
%     d4_a(k) = d4_v(k,1)-d4_v(k-1,1);
% end
% %plot(t, d1_a,  t, d2_a,  t, d3_a,  t, d4_a);
% %plot(t, result1.Var111(start:1:a-1),  t, result1.Var114(start:1:a-1),  t, result1.Var117(start:1:a-1),  t, result1.Var120(start:1:a-1));
% title('dy'); xlabel('Time(s)'); ylabel('velocity(m/s)');legend('d1_y', 'd2_y', 'd3_y', 'd4_y'); grid on; grid minor;
% 
% subplot(2,1,2);
% %plot(t, result1.Var18(start:1:a-1),  t, result1.Var21(start:1:a-1),  t, result1.Var24(start:1:a-1),  t, result1.Var27(start:1:a-1));
% plot(t, result1.Var66(start:1:a-1),  t, result1.Var69(start:1:a-1),  t, result1.Var72(start:1:a-1),  t, result1.Var75(start:1:a-1));
% title('dz'); xlabel('Time(s)'); ylabel('velocity(m/s)');legend('d1_z', 'd2_z', 'd3_z', 'd4_z');
% grid minor;

function eul = Rotation_to_Euler(R)
%ROTATION_TO_EULER Convert a rotation matrix to ZYX Euler angles
%   eul = [roll, pitch, yaw] in radians

    % Ensure orthogonality numerically
    R(:,1) = R(:,1)/norm(R(:,1));
    R(:,2) = R(:,2)/norm(R(:,2));
    R(:,3) = cross(R(:,1), R(:,2));

    % Extract Euler angles (ZYX)
    pitch = asin(-R(3,1));

    if abs(cos(pitch)) > 1e-6
        roll = atan2(R(3,2), R(3,3));
        yaw  = atan2(R(2,1), R(1,1));
    else
        % Gimbal lock
        roll = 0;
        yaw  = atan2(-R(1,2), R(2,2));
    end

    eul = [roll, pitch, yaw];
end

function phi = logm_vec(R)
%LOGM_VEC Convert a rotation matrix to a 3x1 rotation vector (so(3))
%   phi = logm_vec(R) returns the vector such that expm(skew(phi)) = R.

    % numerical safeguard
    if abs(det(R) - 1) > 1e-3
        warning('Input may not be a valid rotation matrix (det != 1)');
    end

    % Clamp trace within [-1, 3]
    tr = max(min(trace(R), 3), -1);
    theta = acos((tr - 1) / 2);

    if abs(theta) < 1e-12
        % Small angle approximation
        phi = 0.5 * [R(3,2) - R(2,3);
                     R(1,3) - R(3,1);
                     R(2,1) - R(1,2)];
    else
        phi = (theta / (2*sin(theta))) * [R(3,2) - R(2,3);
                                          R(1,3) - R(3,1);
                                          R(2,1) - R(1,2)];
    end
end
