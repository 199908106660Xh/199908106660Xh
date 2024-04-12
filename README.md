
clear
clc
close all
% 定义参数
N = 34;  % 订单数量
simulation = 480;  % 假设仿真总时间为480分钟
simtotnum=50;
c = 50;  % 车辆最大载重量, kg
v = 1;  % 配送速度, 千米/分钟
origin = [0, 0];  % 假设商家位于原点
oril = [0, 0]; %车辆决策点实时更新
for simnum=1:simtotnum
    disp(['第',num2str(simnum),'次仿真']);
% 订单生成时间
order_times = rand(N, 1) * simulation;
% 距离分布比例和范围
distance_ratios = [0.40, 0.40, 0.15, 0.05];
distance_ranges = [0, 3; 3, 5; 5, 8; 8, 50];

% 重量分布比例和范围
weight_ratios = [0.718, 0.282];  % 较轻和较重的订单比例
weight_ranges = [1, c * 0.28; c * 0.28, c * 0.72];

% 初始化订单信息数组和坐标矩阵x
orders = struct('time', num2cell(order_times), 'coordinate', [], 'weight', []);
X= zeros(N, 2); % 累积所有订单的坐标
% 初始化订单信息矩阵
orderMatrix = zeros(N, 5);%相当于reqs=[reqr,reql,reqlm，订单编号];
for i = 1:N
    orders(i).coordinate = [0 0]; % 所有需求点的初始坐标都设为原点
end

% 生成订单位置和重量
cumulative_points = 0; % 累积点数，用于追踪生成了多少订单
for i = 1:length(distance_ratios)
     % 如果是最后一个距离范围，则直接使用剩余的订单数量
    if i == length(distance_ratios)
        n_points = N - cumulative_points;
    else
    n_points = round(N * distance_ratios(i)); % 根据比例计算每个范围内的订单数量
    end
    for j = 1:n_points
        angle = 2 * pi * rand();
        r = distance_ranges(i, 1) + (distance_ranges(i, 2) - distance_ranges(i, 1)) * rand();
        % 生成随机坐标
         coord = origin + r * [cos(angle), sin(angle)];
        orders(cumulative_points + j).coordinate = coord;
        X(cumulative_points + j, :) = coord; % 添加坐标到x矩阵
        % 决定订单重量
        weight_dist_index = rand() < weight_ratios(2);
        weight = weight_ranges(weight_dist_index + 1, 1) + ...
                (weight_ranges(weight_dist_index + 1, 2) - weight_ranges(weight_dist_index + 1, 1)) * rand();
        
        orders(cumulative_points + j).weight = weight;
         % 更新订单的索引
        index = cumulative_points + j;
         if index > N
            error('Index exceeds the number of orders. Something went wrong with index calculations.');
        end
         % 存储订单信息到orderMatrix中
        orderMatrix(index, 1) = order_times(index);         % 订单时间
        orderMatrix(index, 2:3) = coord;             % 订单坐标
        orderMatrix(index, 4) = weight;                      % 订单重量
    end
    cumulative_points = cumulative_points + n_points;
end
% 计算距离矩阵
D = squareform(pdist(X, 'euclidean'));
% 打乱顺序以消除分布偏差
shuffled_indices = randperm(N);
orders = orders(shuffled_indices);
orderMatrix = orderMatrix(shuffled_indices, :); % 也重新排列orderMatrix以保持一致

% 排序订单并添加编号
[~, sortedIdx] = sort([orders.time]);% 获取按时间排序的索引
sortedOrders = orders(sortedIdx); % 使用索引排序结构数组
orderMatrix = orderMatrix(sortedIdx, :); % 重新排列orderMatrix

% 添加编号到orderMatrix
orderMatrix(:, 5) = 1:N;
% 打开文件准备写入
fileID = fopen('orders.txt', 'w');%数据写入名为 orders.txt 文件
fprintf(fileID, '订单信息：\n');

% 将排序且编号的订单信息保存到文件
for idx = 1:N
     % 添加编号信息，编号为排序后的顺序
    sortedOrders(idx).number = idx;
   % 打印到控制台
    disp(['订单时间: ', num2str(sortedOrders(idx).time), ...
          ' 坐标: (', num2str(sortedOrders(idx).coordinate(1)), ...
          ',', num2str(sortedOrders(idx).coordinate(2)), ...
          ') 重量: ', num2str(sortedOrders(idx).weight), ...
          ' kg 编号: ', num2str(idx)]);
    
    % 写入到文件
    fprintf(fileID, '订单时间: %.2f, 坐标: (%.2f, %.2f), 重量: %.2f kg, 编号: %d\n', ...
            sortedOrders(idx).time, ...
            sortedOrders(idx).coordinate(1), sortedOrders(idx).coordinate(2), ...
            sortedOrders(idx).weight, sortedOrders(idx).number);
end
% 关闭文件
fclose(fileID);
%初始（0,0），在服务完一个需求点之后，把该点的坐标作为新的起始点或原点，并保持后续的计算或操作以这个新原点作为参照点
maxdis=max(D(1,:));        %网络结构特征，离原点的最远距离。
avedis=mean(D(1,:));       %网络结构特征，离原点的平均距离。
fprintf('最远距离 (maxdis): %f\n', maxdis);
fprintf('平均距离 (avedis): %f\n', avedis);
objJPL=zeros(simtotnum,1);
objWS=zeros(simtotnum,1);
offlineobj=zeros(simtotnum,1);
JPLratio=zeros(simtotnum,1);
WSratio=zeros(simtotnum,1);
JPLWSratio=zeros(simtotnum,1);
[serroutJPL, sertimeJPL,objJPL(simnum)]=JPL(orderMatrix,oril,v);
% 显示调用JPL函数后得到的结果
fprintf(' (serroutJPL): %f\n', serroutJPL);
fprintf(' (sertimeJPL): %f\n', sertimeJPL);
fprintf(' (objJPL) 对于仿真 #%d: %f\n', simnum, objJPL(simnum));

% 画出配送员路径图
figure;
hold on
plot(X(:,1),X(:,2),'o','color',[0.5,0.5,0.5])
for i=1:size(X,1)
    text(X(i,1)+0.05,X(i,2)+0.05,num2str(i),'color',[1,0,0]);
end
%如果serroutJPL包含足够的列来表示x和y坐标，希望绘制的路径的起始和结束点
if size(serroutJPL, 2) >= 2
    for i=2:size(serroutJPL,1)
        [arrowx,arrowy] = dsxy2figxy(gca,serroutJPL(i-1:i,1),serroutJPL(i-1:i,2));%坐标转换
        annotation('textarrow',arrowx,arrowy,'HeadWidth',8,'color',[0,0,1]);
    end
end
row=size(serroutJPL,1);
for i=2:row
    [arrowx,arrowy] = dsxy2figxy(gca,serroutJPL(i-1:i,1),serroutJPL(i-1:i,2));%坐标转换
    annotation('textarrow',arrowx,arrowy,'HeadWidth',8,'color',[0,0,1]);
end
hold off
xlabel('横坐标')
ylabel('纵坐标')
title('轨迹图')
box on
%% 输出解的路线和总距离
%输出JPL策略的结果
disp('JPL策略下配送员行驶路线（时间）为:')
NJPL=size(serroutJPL,1);
pJPL=[];
for i=1:NJPL
    pJPL=[pJPL,'—>',num2str(serroutJPL(i,3)),'(',num2str(sertimeJPL(i)),')'];    
end
disp(pJPL)
disp(['总时间：',num2str(objJPL(simnum))]);

[serroutWS, sertimeWS,objWS(simnum)]=WS(orderMatrix,oril,c,v);%sortedOrders);
%输出Wait & Serve策略的结果
disp('Wait & Serve策略下配送员行驶路线（时间）为:')
NWS=size(serroutWS,1);
pWS=[];
for i=1:NWS
    pWS=[pWS,'—>',num2str(serroutWS(i,3)),'(',num2str(sertimeWS(i)),')'];    
end
disp(pWS)
disp(['总时间：',num2str(objWS(simnum))]);

%%输出离线解的下界，最后一个需求点释放时间+距离
disp('离线解的下界为:')
for i=1:N
    offlineobj(simnum)=offlineobj(simnum)+orderMatrix(i,1)+D(1,orderMatrix(i,5))/v;
end
disp(num2str(offlineobj(simnum)))
disp('JPL策略与离线解下界之比为：')
JPLratio(simnum)=objJPL(simnum)/offlineobj(simnum);
disp(JPLratio(simnum))
disp('Wait & Serve策略与离线解下界之比为：')
WSratio(simnum)=objWS(simnum)/offlineobj(simnum);
disp(WSratio(simnum))
disp('JPL策略与Wait & Serve策略目标函数值之比为：')
JPLWSratio(simnum)=objJPL(simnum)/objWS(simnum);
disp(JPLWSratio(simnum))
disp('-------------------------------------------------------------')
end
