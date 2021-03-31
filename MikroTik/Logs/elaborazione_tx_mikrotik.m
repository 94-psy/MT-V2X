clear all; close all; clc
%d = acos(cosd(90-Lat(1:end-1)) .* cosd(90-Lat(2:end)) + sind(90-Lat(1:end-1)) .* sind(90-Lat(2:end))) .* cosd(Lon(1:end-1)-Lon(2:end)) * 6378137;

currentFolder = pwd;


filenametx = 'log_TX.txt';
filenamerx = 'log_RX.txt';

rawdatatx=readtable(filenametx);
rawdatarx=readtable(filenamerx);

% TX DATA FROM TX FILE
TXUTC_raw=rawdatatx.UTC;
TXlatitude_raw=rawdatatx.Latitude_raw;
TXlatitude_raw_dir=rawdatatx.Latitude_dir;
TXlongitude_raw=rawdatatx.Longitude_raw;
TXlongitude_raw_dir=rawdatatx.Longitude_dir;
TXspeed_raw=(rawdatatx.speed_raw)/1.9438444924406;
TXheading_raw=rawdatatx.heading_raw;
TXseq_num=rawdatatx.seq_num;
TXtrig_S=rawdatatx.trig_S;
TXtrig_H=rawdatatx.trig_H;
TXtrig_D=rawdatatx.trig_D;
TXtrig_TO=rawdatatx.trig_TO;

% RX AND TX DATA FROM RX FILE

RXUTC_tx_raw=rawdatarx.UTC_TX;
RXLatitude_tx_raw=rawdatarx.Latitude_raw_TX;
RXLatitude_tx_raw_dir=rawdatarx.Latitude_dir_TX;
RXLongitude_tx_raw=rawdatarx.Longitude_raw_TX;
RXLongitude_tx_raw_dir=rawdatarx.Longitude_dir_TX;
RXspeed_raw_tx=(rawdatarx.speed_raw_TX)/1.9438444924406;
RXheading_raw_tx=rawdatarx.heading_raw_TX;
RXseq_num_tx=rawdatarx.seq_num_TX;
RXUTC_raw=rawdatarx.UTC_RX;
RXLatitude_raw=rawdatarx.Latitude_raw_RX;
RXLatitude_raw_dir=rawdatarx.Latitude_dir_RX;
RXLongitude_raw=rawdatarx.Longitude_raw_RX;
RXLongitude_raw_dir=rawdatarx.Longitude_dir_RX;
RXspeed_raw=(rawdatarx.speed_raw_RX)/1.9438444924406;
RXheading_raw=rawdatarx.heading_raw_RX;
IPG=rawdatarx.IPG;
latency=rawdatarx.Latency;
Distance=rawdatarx.Distance_RXTX;
RXUTC_gps_raw=rawdatarx.UTC_RAW_GPS;
diff=rawdatarx.diff;
TXlatitude_dir=zeros(length(TXlatitude_raw_dir),1);
TXlongitude_dir=zeros(length(TXlatitude_raw_dir),1);
% TXlatitude_dir=zeros(length(TXlatitude_raw_dir),1);
% TXlatitude_dir=zeros(length(TXlatitude_raw_dir),1);

for i=1:1:length(TXlatitude_raw_dir)

    if isequal(TXlatitude_raw_dir{i},'N')
        TXlatitude_dir(i)=1;
    else
        TXlatitude_dir(i)=-1;
    end
    if isequal(TXlongitude_raw_dir{i},'E')
        TXlongitude_dir(i)=1;
    else
        TXlongitude_dir(i)=-1;
    end

end


h = fix(TXUTC_raw/10000);
m = fix((TXUTC_raw - 10000*h)/100);
s = TXUTC_raw - 10000*h - 100*m;
TXUTC=(h*3600+m*60+s)*1000;
h = fix(RXUTC_raw/10000);
m = fix((RXUTC_raw - 10000*h)/100);
s = RXUTC_raw - 10000*h - 100*m;
RXUTC=(h*3600+m*60+s)*1000;
h = fix(RXUTC_gps_raw/10000);
m = fix((RXUTC_gps_raw - 10000*h)/100);
s = RXUTC_gps_raw - 10000*h - 100*m;
RXUTC_gps=(h*3600+m*60+s)*1000;
h = fix(RXUTC_tx_raw/10000);
m = fix((RXUTC_tx_raw - 10000*h)/100);
s = RXUTC_tx_raw - 10000*h - 100*m;
RXUTC_tx=(h*3600+m*60+s)*1000;

tmp=fix(TXlatitude_raw/100)*100;
TXlatitude= (tmp/100+((TXlatitude_raw-tmp)/60)).*TXlatitude_dir;
tmp=fix(TXlongitude_raw/100)*100;
TXlongitude= (tmp/100+((TXlongitude_raw-tmp)/60)).*TXlongitude_dir;
clear h; clear m; clear s; clear tmp;




RXUTC_tx_n=zeros(length(TXseq_num),1);
RXUTC_n=zeros(length(TXseq_num),1);
RXUTC_gps_n=zeros(length(TXseq_num),1);

RXLatitude_tx_raw_n=zeros(length(TXseq_num),1);
RXLatitude_tx_raw_dir_n=zeros(length(TXseq_num),1);
RXLongitude_tx_raw_n=zeros(length(TXseq_num),1);
RXLongitude_tx_raw_dir_n=zeros(length(TXseq_num),1);
RXspeed_raw_tx_n=zeros(length(TXseq_num),1);
RXheading_raw_tx_n=zeros(length(TXseq_num),1);
RXseq_num_tx_n=zeros(length(TXseq_num),1);
RXLatitude_raw_n=zeros(length(TXseq_num),1);
RXLatitude_raw_dir_n=zeros(length(TXseq_num),1);
RXLongitude_raw_n=zeros(length(TXseq_num),1);
RXLongitude_raw_dir_n=zeros(length(TXseq_num),1);
RXspeed_raw_n=zeros(length(TXseq_num),1);
RXheading_raw_n=zeros(length(TXseq_num),1);
RXLatitude_dir=zeros(length(RXLatitude_raw_dir),1);
RXLongitude_dir=zeros(length(RXLatitude_raw_dir),1);
RXlatitude_tx_dir=zeros(length(RXLatitude_raw_dir),1);
RXlongitude_tx_dir=zeros(length(RXLatitude_raw_dir),1);
latency_n=zeros(length(RXLatitude_raw_dir),1);
for i=1:1:length(RXLatitude_raw_dir)

    if isequal(RXLatitude_raw_dir{i},'N')
        RXLatitude_dir(i)=1;
    else
        RXLatitude_dir(i)=-1;
    end
    if isequal(RXLongitude_raw_dir{i},'E')
        RXLongitude_dir(i)=1;
    else
        RXLongitude_dir(i)=-1;
    end
    if isequal(RXLatitude_tx_raw_dir{i},'N')
        RXlatitude_tx_dir(i)=1;
    else
        RXlatitude_tx_dir(i)=-1;
    end
    if isequal(RXLongitude_tx_raw_dir{i},'E')
        RXlongitude_tx_dir(i)=1;
    else
        RXlongitude_tx_dir(i)=-1;
    end
end
k=1;
for kk=1:1:length(TXseq_num)
    if k > length(RXUTC_tx)
        RXUTC_tx_n(kk)=NaN;
        RXUTC_n(kk)=NaN;
        RXUTC_gps_n(kk)=NaN;
        RXLatitude_tx_raw_n(kk)=NaN;
        RXLatitude_tx_raw_dir_n(kk)=NaN;
        RXLongitude_tx_raw_n(kk)=NaN;
        RXLongitude_tx_raw_dir_n(kk)=NaN;
        RXspeed_raw_tx_n(kk)=NaN;
        RXheading_raw_tx_n(kk)=NaN;
        RXseq_num_tx_n(kk)=NaN;
        RXLatitude_raw_n(kk)=NaN;
        RXLatitude_raw_dir_n(kk)=NaN;
        RXLongitude_raw_n(kk)=NaN;
        RXLongitude_raw_dir_n(kk)=NaN;
        RXspeed_raw_n(kk)=NaN;
        RXheading_raw_n(kk)=NaN;
        IPG_n(kk)=NaN;
        Distance_n(kk)=NaN;
        latency_n(kk)=NaN;
    else
        if TXseq_num(kk) == RXseq_num_tx(k)
            %ho ricevuto il pacchetto, NaN
            
            RXUTC_tx_n(kk)=RXUTC_tx(k);
            RXUTC_n(kk)=RXUTC(k);
            RXUTC_gps_n(kk)=RXUTC_gps(k);
            RXLatitude_tx_raw_n(kk)=RXLatitude_tx_raw(k);
            RXLatitude_tx_raw_dir_n(kk)=RXlatitude_tx_dir(k);
            RXLongitude_tx_raw_n(kk)=RXLongitude_tx_raw(k);
            RXLongitude_tx_raw_dir_n(kk)=RXlongitude_tx_dir(k);
            RXspeed_raw_tx_n(kk)=RXspeed_raw_tx(k);
            RXheading_raw_tx_n(kk)=RXheading_raw_tx(k);
            RXseq_num_tx_n(kk)=RXseq_num_tx(k);
            RXLatitude_raw_n(kk)=RXLatitude_raw(k);
            RXLatitude_raw_dir_n(kk)=RXLatitude_dir(k);
            RXLongitude_raw_n(kk)=RXLongitude_raw(k);
            RXLongitude_raw_dir_n(kk)=RXLongitude_dir(k);
            RXspeed_raw_n(kk)=RXspeed_raw(k);
            RXheading_raw_n(kk)=RXheading_raw(k);
            IPG_n(kk)=IPG(k);
            Distance_n(kk)=Distance(k);
            latency_n(kk)=latency(k);
            k=k+1;
            
        else
            % ho ricevuto il pacchetto, NaN
            RXUTC_tx_n(kk)=NaN;
            RXUTC_n(kk)=NaN;
            RXUTC_gps_n(kk)=NaN;
            RXLatitude_tx_raw_n(kk)=NaN;
            RXLatitude_tx_raw_dir_n(kk)=NaN;
            RXLongitude_tx_raw_n(kk)=NaN;
            RXLongitude_tx_raw_dir_n(kk)=NaN;
            RXspeed_raw_tx_n(kk)=NaN;
            RXheading_raw_tx_n(kk)=NaN;
            RXseq_num_tx_n(kk)=NaN;
            RXLatitude_raw_n(kk)=NaN;
            RXLatitude_raw_dir_n(kk)=NaN;
            RXLongitude_raw_n(kk)=NaN;
            RXLongitude_raw_dir_n(kk)=NaN;
            RXspeed_raw_n(kk)=NaN;
            RXheading_raw_n(kk)=NaN;
            IPG_n(kk)=NaN;
            Distance_n(kk)=NaN;
            latency_n(kk)=NaN;
        end
    end
    %     kk=kk+1;
    
end

Latency=(RXUTC_n-TXUTC);
% ricalcolo IPG

tmp=fix(RXLatitude_tx_raw_n/100)*100;
RXlatitude_tx= (tmp/100+((RXLatitude_tx_raw_n-tmp)/60)).*RXLatitude_tx_raw_dir_n;
tmp=fix(RXLongitude_tx_raw_n/100)*100;
RXlongitude_tx= (tmp/100+((RXLongitude_tx_raw_n-tmp)/60)).*RXLongitude_tx_raw_dir_n;
tmp=fix(RXLatitude_raw_n/100)*100;
RXlatitude= (tmp/100+((RXLatitude_raw_n-tmp)/60)).*RXLatitude_raw_dir_n;
tmp=fix(RXLongitude_raw_n/100)*100;
RXlongitude= (tmp/100+((RXLongitude_raw_n-tmp)/60)).*RXLongitude_raw_dir_n;

clear h; clear m; clear s; clear tmp;
D = deg2km(distance(TXlatitude,TXlongitude,RXlatitude,RXlongitude))*1000;

hit=0;
miss=0;
loss=zeros(1,length(TXseq_num));

for k=1:1:length(RXseq_num_tx_n)
    if isnan(RXseq_num_tx_n(k))
      miss=miss+1;
    end
    loss(k)=(miss/TXseq_num(k))*100;
    
end


ipg_interval=zeros(11,1);
for i=1:length(IPG)
    if IPG(i) < 100
    ipg_interval(1)=ipg_interval(1)+1;
    end
    if  IPG(i) >= 100 && IPG(i) < 200
    ipg_interval(2)=ipg_interval(2)+1;
    end
    if IPG(i) >= 200 && IPG(i) < 300
    ipg_interval(3)=ipg_interval(3)+1;
    end
    if IPG(i) >= 300 && IPG(i) < 400
    ipg_interval(4)=ipg_interval(4)+1;
    end
    if IPG(i) >= 400 && IPG(i) < 500
    ipg_interval(5)=ipg_interval(5)+1;
    end
    if IPG(i) >= 500 && IPG(i) < 600
    ipg_interval(6)=ipg_interval(6)+1;
    end
    if IPG(i) >= 600 && IPG(i) < 700
    ipg_interval(7)=ipg_interval(7)+1;
    end
    if IPG(i) >= 700 && IPG(i) < 800
    ipg_interval(8)=ipg_interval(8)+1;
    end
    if IPG(i) >= 800 && IPG(i) < 900
    ipg_interval(9)=ipg_interval(9)+1;
    end
    if IPG(i) >= 900 && IPG(i) < 1000
    ipg_interval(10)=ipg_interval(10)+1;
    end
    if IPG(i) > 1000
    ipg_interval(11)=ipg_interval(11)+1;
    end
end
ipg_interval=(ipg_interval/length(IPG))*100;
ipg_interval_xaxis=100:100:1100;

F_ipg(1)=ipg_interval(1);
F_ipg(2)=ipg_interval(1)+ipg_interval(2);
F_ipg(3)=ipg_interval(1)+ipg_interval(2)+ipg_interval(3);
F_ipg(4)=ipg_interval(1)+ipg_interval(2)+ipg_interval(3)+ipg_interval(4);
F_ipg(5)=ipg_interval(1)+ipg_interval(2)+ipg_interval(3)+ipg_interval(4)+ipg_interval(5);
F_ipg(6)=ipg_interval(1)+ipg_interval(2)+ipg_interval(3)+ipg_interval(4)+ipg_interval(5)+ipg_interval(6);
F_ipg(7)=ipg_interval(1)+ipg_interval(2)+ipg_interval(3)+ipg_interval(4)+ipg_interval(5)+ipg_interval(6)+ipg_interval(7)+ipg_interval(8);
F_ipg(8)=ipg_interval(1)+ipg_interval(2)+ipg_interval(3)+ipg_interval(4)+ipg_interval(5)+ipg_interval(6)+ipg_interval(7)+ipg_interval(8)+ipg_interval(9);
F_ipg(9)=ipg_interval(1)+ipg_interval(2)+ipg_interval(3)+ipg_interval(4)+ipg_interval(5)+ipg_interval(6)+ipg_interval(7)+ipg_interval(8)+ipg_interval(9)+ipg_interval(10);
F_ipg(10)=ipg_interval(1)+ipg_interval(2)+ipg_interval(3)+ipg_interval(4)+ipg_interval(5)+ipg_interval(6)+ipg_interval(7)+ipg_interval(8)+ipg_interval(9)+ipg_interval(10);
F_ipg(11)=ipg_interval(1)+ipg_interval(2)+ipg_interval(3)+ipg_interval(4)+ipg_interval(5)+ipg_interval(6)+ipg_interval(7)+ipg_interval(8)+ipg_interval(9)+ipg_interval(10)+ipg_interval(11);

% figure('Name','MAP')
% for j=1:1:(length(TXlatitude)-1)
%    geoplot([TXlatitude(j) TXlatitude(j+1)],[TXlongitude(j) TXlongitude(j+1)], 'r'); hold on
% %    geoscatter(TXlatitude(j),TXlongitude(j),'r'); hold on
% end



