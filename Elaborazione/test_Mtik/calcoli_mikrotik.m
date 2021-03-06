clear all; close all; clc
currentFolder = pwd;

%% test
% addpath('04032021\confronto\')
Mtest1filerx = 'log_RX_26_10_34_02.txt';
Mtest1filetx = 'log_TX_26_10_34_06.txt';
Mtest2filerx = 'log_RX_26_13_07_27.txt';
Mtest2filetx = 'log_TX_26_13_07_33.txt';
Mtest3filerx = 'log_RX_26_16_39_54.txt';
Mtest3filetx = 'log_TX_26_16_39_49.txt';

%% READ DATA FROM FILES
Mtest1rx=readtable(Mtest1filerx);
Mtest1tx=readtable(Mtest1filetx);
Mtest2rx=readtable(Mtest2filerx);
Mtest2tx=readtable(Mtest2filetx);
Mtest3rx=readtable(Mtest3filerx);
Mtest3tx=readtable(Mtest3filetx);


%% MIKROTIK_DATA



% TX DATA FROM TX FILE

MTX1millis=(Mtest1tx.Timestamp(1:5000))/1000; %in ms
MTX1seq_num=Mtest1tx.seq_num(1:5000);
MTX2millis=(Mtest2tx.Timestamp(1:5000))/1000; %in ms
MTX2seq_num=Mtest2tx.seq_num(1:5000);
MTX3millis=(Mtest3tx.Timestamp(1:5000))/1000; %in ms
MTX3seq_num=Mtest3tx.seq_num(1:5000);



% RX AND TX DATA FROM RX FILE

MRX1millis_TX=(Mtest1rx.TimestampTX)/1000; %in ms
MRX1millis=(Mtest1rx.Timestamp_RX)/1000; %in ms
MRX1seq_num_TX=Mtest1rx.seq_num_TX;

MRX2millis_TX=(Mtest2rx.TimestampTX)/1000; %in ms
MRX2millis=(Mtest2rx.Timestamp_RX)/1000; %in ms
MRX2seq_num_TX=Mtest2rx.seq_num_TX;

MRX3millis_TX=(Mtest3rx.TimestampTX)/1000; %in ms
MRX3millis=(Mtest3rx.Timestamp_RX)/1000; %in ms
MRX3seq_num_TX=Mtest3rx.seq_num_TX;



%% MIKROTIK_COMPUTATION



%preparo le variabili

MRX1millis_TX_n=zeros(length(MTX1seq_num),1);
MRX1millis_n=zeros(length(MTX1seq_num),1);
MRX1seq_num_TX_n=zeros(length(MTX1seq_num),1);

MRX2millis_TX_n=zeros(length(MTX2seq_num),1);
MRX2millis_n=zeros(length(MTX2seq_num),1);
MRX2seq_num_TX_n=zeros(length(MTX2seq_num),1);

MRX3millis_TX_n=zeros(length(MTX3seq_num),1);
MRX3millis_n=zeros(length(MTX3seq_num),1);
MRX3seq_num_TX_n=zeros(length(MTX3seq_num),1);


%riallineo i dati del RX con i dati del TX
k=1;
for j=1:1:length(MTX1seq_num)
     if MTX1seq_num(j) == MRX1seq_num_TX(k)
         MRX1millis_TX_n(j)=MRX1millis_TX(k);
         MRX1millis_n(j)=MRX1millis(k);
         MRX1seq_num_TX_n(j)= MRX1seq_num_TX(k);
         k=k+1;
     else
         MRX1millis_TX_n(j)=NaN;
         MRX1millis_n(j)=NaN;
         MRX1seq_num_TX_n(j)=NaN;
     end
end
k=1;
for j=1:1:length(MTX2seq_num)
     if MTX2seq_num(j) == MRX2seq_num_TX(k)
         MRX2millis_TX_n(j)=MRX2millis_TX(k);
         MRX2millis_n(j)=MRX2millis(k);
         MRX2seq_num_TX_n(j)= MRX2seq_num_TX(k);
         k=k+1;
     else
         MRX2millis_TX_n(j)=NaN;
         MRX2millis_n(j)=NaN;
         MRX2seq_num_TX_n(j)=NaN;
     end
end
k=1;
for j=1:1:length(MTX3seq_num)
     if MTX3seq_num(j) == MRX3seq_num_TX(k)
         MRX3millis_TX_n(j)=MRX3millis_TX(k);
         MRX3millis_n(j)=MRX3millis(k);
         MRX3seq_num_TX_n(j)= MRX3seq_num_TX(k);
         k=k+1;
     else
         MRX3millis_TX_n(j)=NaN;
         MRX3millis_n(j)=NaN;
         MRX3seq_num_TX_n(j)=NaN;
     end
end

%calcolo la latenza

Mlatency1=(MRX1millis_n-MRX1millis_TX_n);
Mlatency2=(MRX2millis_n-MRX2millis_TX_n);
Mlatency3=(MRX3millis_n-MRX3millis_TX_n);

MLatency_interval1=abs(Mlatency1(1)-Mlatency1(end))/length(Mlatency1);

MLatency_interval2=abs(Mlatency2(1)-Mlatency2(end))/length(Mlatency2);



% %correzione della latenza per raddrizzare ulteriormente i valori
for i=1:1:length(Mlatency1)
    if Mlatency1(1) < Mlatency1(end)
        Mlatency1_corr(i)=Mlatency1(i)-MLatency_interval1*(i-1);
    else
        Mlatency1_corr(i)=Mlatency1(i)+MLatency_interval1*(i-1);
    end
end
for i=1:1:length(Mlatency2)
    if Mlatency2(1) < Mlatency2(end)
        Mlatency2_corr(i)=Mlatency2(i)-MLatency_interval2*(i-1);
    else
        Mlatency2_corr(i)=Mlatency2(i)+MLatency_interval2*(i-1);
    end
end


%calcolo perdite
Mhit1=0;
Mmiss1=0;
Mloss1=zeros(1,length(MTX1seq_num));
Mhit2=0;
Mmiss2=0;
Mloss2=zeros(1,length(MTX2seq_num));
Mhit3=0;
Mmiss3=0;
Mloss3=zeros(1,length(MTX3seq_num));


for k=1:1:length(MTX1seq_num)
    if isnan(MRX1seq_num_TX_n(k))
        Mmiss1=Mmiss1+1;
    end
    Mloss1(k)=(Mmiss1/MTX1seq_num(k))*100;
end
for k=1:1:length(MTX2seq_num)
    if isnan(MRX2seq_num_TX_n(k))
        Mmiss2=Mmiss2+1;
    end
    Mloss2(k)=(Mmiss2/MTX2seq_num(k))*100;
end
for k=1:1:length(MTX3seq_num)
    if isnan(MRX3seq_num_TX_n(k))
        Mmiss3=Mmiss3+1;
    end
    Mloss3(k)=(Mmiss3/MTX3seq_num(k))*100;
end


%calcolo PER
i=1;
per_index=1;
Muscita=false(1);
while ~Muscita
    MMISS_PER=0;
    last_i=i;
    Mlast_timestamp=MTX1millis(i);
    num_pacchetti=0;
    while ( MTX1millis(i)- Mlast_timestamp) < 5*10^3
        if isnan(MRX1seq_num_TX_n(i))
            MMISS_PER=MMISS_PER+1;
            num_pacchetti=num_pacchetti+1;
        else
            num_pacchetti=num_pacchetti+1;
        end
        i=i+1;
    end
    MPER1(per_index)=MMISS_PER/num_pacchetti;
    per_index=per_index+1;
    i=last_i+1;
    Mtest_i=i+50;
    if Mtest_i >= length(MRX1seq_num_TX_n)
        Muscita=true(1);
    end
end


%calcolo PER
i=1;
per_index=1;
Muscita=false(1);
while ~Muscita
    MMISS_PER=0;
    last_i=i;
    Mlast_timestamp=MTX2millis(i);
    num_pacchetti=0;
    while ( MTX2millis(i)- Mlast_timestamp) < 5*10^3
        if isnan(MRX2seq_num_TX_n(i))
            MMISS_PER=MMISS_PER+1;
            num_pacchetti=num_pacchetti+1;
        else
            num_pacchetti=num_pacchetti+1;
        end
        i=i+1;
    end
    MPER2(per_index)=MMISS_PER/num_pacchetti;
    per_index=per_index+1;
    i=last_i+1;
    Mtest_i=i+50;
    if Mtest_i >= length(MRX2seq_num_TX_n)
        Muscita=true(1);
    end
end


%calcolo PER
i=1;
per_index=1;
Muscita=false(1);
while ~Muscita
    MMISS_PER=0;
    last_i=i;
    Mlast_timestamp=MTX3millis(i);
    num_pacchetti=0;
    while ( MTX3millis(i)- Mlast_timestamp) < 5*10^3
        if isnan(MRX3seq_num_TX_n(i))
            MMISS_PER=MMISS_PER+1;
            num_pacchetti=num_pacchetti+1;
        else
            num_pacchetti=num_pacchetti+1;
        end
        i=i+1;
    end
    MPER3(per_index)=MMISS_PER/num_pacchetti;
    per_index=per_index+1;
    i=last_i+1;
    Mtest_i=i+50; 
    if Mtest_i >= length(MRX3seq_num_TX_n)
        Muscita=true(1);
    end
end



%% MIKROTIK_GRAPH

figure('Name','LANTENCY')
subplot(3,1,1)
xaxis=1:1:length(Mlatency1_corr);
plot(xaxis,Mlatency1_corr,'b'), grid on
xlim([0 length(xaxis)])
ylabel('Latency [ms]')
xlabel('# Packets')
subplot(3,1,2)
xaxis=1:1:length(Mlatency2_corr);
plot(xaxis,Mlatency2_corr,'b'), grid on
xlim([0 length(xaxis)])
ylabel('Latency [ms]')
xlabel('# Packets')
subplot(3,1,3)
xaxis=1:1:length(Mlatency3);
plot(xaxis,Mlatency3,'b'), grid on
xlim([0 length(xaxis)])
ylabel('Latency [ms]')
xlabel('# Packets')

figure('Name','LOSS')
subplot(3,1,1)
xaxis=1:1:length(Mloss1);
plot(xaxis,Mloss1,'b'), grid on
xlim([0 length(xaxis)])
ylabel('Loss [%]')
xlabel('# Packets')
subplot(3,1,2)
xaxis=1:1:length(Mloss2);
plot(xaxis,Mloss2,'b'), grid on
xlim([0 length(xaxis)])
ylabel('Loss [%]')
xlabel('# Packets')
subplot(3,1,3)
xaxis=1:1:length(Mloss3);
plot(xaxis,Mloss3,'b'), grid on
xlim([0 length(xaxis)])
ylabel('Loss [%]')
xlabel('# Packets')

figure('Name','PER')
subplot(3,1,1)
xaxis=1:1:length(MPER1);
plot(xaxis,MPER1*100,'b'), grid on
xlim([0 length(xaxis)])
ylabel('PER [%]')
xlabel('# Packets')
subplot(3,1,2)
xaxis=1:1:length(MPER2);
plot(xaxis,MPER2*100,'b'), grid on
xlim([0 length(xaxis)])
ylabel('PER [%]')
xlabel('# Packets')
subplot(3,1,3)
xaxis=1:1:length(MPER3);
plot(xaxis,MPER3*100,'b'), grid on
xlim([0 length(xaxis)])
ylabel('PER [%]')
xlabel('# Packets')


% geoscatter(44.627900,10.948880,'y','filled'); hold on ; geoscatter(44.628198,10.948000,'b','filled'); geobasemap satellite
% geoplot([44.627900 44.628198],[10.948880 10.94800],'w')


geoscatter(44.628716,10.947609,'y','filled'); hold on ; geoscatter(44.628257,10.948555,'b','filled'); geobasemap satellite
geoplot([44.628716 44.628257],[10.947609 ,10.948555],'w') %91.5 metri
