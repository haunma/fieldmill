N_CHUNKS = 5000;
%I_SAMPLES_PER_CHUNK = 96e3 / 1 / 100;
I_SAMPLES_PER_CHUNK = 2.4e6 / 25 / 100;
FIR_SAMPLES_PER_CHUNK = I_SAMPLES_PER_CHUNK / 8;
OUT_SAMPLES_PER_CHUNK = FIR_SAMPLES_PER_CHUNK / 8;
%TOTAL_SAMPLES = N_CHUNKS * 2 * I_SAMPLES_PER_CHUNK;

%s = serial('COM6', 'BaudRate', 9600, 'InputBufferSize', TOTAL_BYTES, 'Timeout', 120);
s = serial('COM6', 'BaudRate', 9600, 'InputBufferSize', 10e6, 'Timeout', 120);
fopen(s);

%d = fread(s, TOTAL_SAMPLES, 'int16');

%I = zeros(1, N_CHUNKS * I_SAMPLES_PER_CHUNK);
%Q = zeros(1, N_CHUNKS * I_SAMPLES_PER_CHUNK);
FI = zeros(1, N_CHUNKS * FIR_SAMPLES_PER_CHUNK);
FQ = zeros(1, N_CHUNKS * FIR_SAMPLES_PER_CHUNK);
outI = zeros(1, N_CHUNKS * OUT_SAMPLES_PER_CHUNK);
outQ = zeros(1, N_CHUNKS * OUT_SAMPLES_PER_CHUNK);
index = zeros(1, N_CHUNKS);
indexInd = 1;

for k = 1:N_CHUNKS
    if mod(k,10) == 0
        k
    end
    %I((k-1)*I_SAMPLES_PER_CHUNK + [1:I_SAMPLES_PER_CHUNK]) = d((k-1)*2*I_SAMPLES_PER_CHUNK + [1:I_SAMPLES_PER_CHUNK]);
    %Q((k-1)*I_SAMPLES_PER_CHUNK + [1:I_SAMPLES_PER_CHUNK]) = d((k-1)*2*I_SAMPLES_PER_CHUNK + I_SAMPLES_PER_CHUNK + [1:I_SAMPLES_PER_CHUNK]);

    d1 = fread(s, 2*FIR_SAMPLES_PER_CHUNK, 'int16');
    d2 = fread(s, 2*OUT_SAMPLES_PER_CHUNK, 'float');
    d3 = fread(s, 8, 'uint32');
    FI((k-1)*FIR_SAMPLES_PER_CHUNK + [1:FIR_SAMPLES_PER_CHUNK]) = d1([1:FIR_SAMPLES_PER_CHUNK]);
    FQ((k-1)*FIR_SAMPLES_PER_CHUNK + [1:FIR_SAMPLES_PER_CHUNK]) = d1(FIR_SAMPLES_PER_CHUNK + [1:FIR_SAMPLES_PER_CHUNK]);
    outI((k-1)*OUT_SAMPLES_PER_CHUNK + [1:OUT_SAMPLES_PER_CHUNK]) = d2([1:OUT_SAMPLES_PER_CHUNK]);
    outQ((k-1)*OUT_SAMPLES_PER_CHUNK + [1:OUT_SAMPLES_PER_CHUNK]) = d2(OUT_SAMPLES_PER_CHUNK + [1:OUT_SAMPLES_PER_CHUNK]);
    
    if d3(1) ~= 0
        index(indexInd) = d3(1);
        indexInd = indexInd + 1;
    end
    
    p = 2;
    while p <= 8 && d3(p) > 0
        index(indexInd) = d3(p);
        indexInd = indexInd + 1;
        p = p + 1;
    end
end    

fclose(s);
