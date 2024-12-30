module regfile(data_in,writenum,write,readnum,clk,data_out);
input [15:0] data_in;
input [2:0] writenum, readnum;
input write, clk;
output reg [15:0] data_out;

//My signals:
wire [7:0] dc_write_out;
wire [15:0] R0, R1, R2, R3, R4, R5, R6, R7;

//Write decoder 
decoder dc_write (.in(writenum), .out(dc_write_out));

//Load enable for each register
load_enable l0 (.in(data_in), .load(write), .write_bit(dc_write_out[0]), .clk(clk), .out(R0[15:0]));
load_enable l1 (.in(data_in), .load(write), .write_bit(dc_write_out[1]), .clk(clk), .out(R1[15:0]));
load_enable l2 (.in(data_in), .load(write), .write_bit(dc_write_out[2]), .clk(clk), .out(R2[15:0]));
load_enable l3 (.in(data_in), .load(write), .write_bit(dc_write_out[3]), .clk(clk), .out(R3[15:0]));
load_enable l4 (.in(data_in), .load(write), .write_bit(dc_write_out[4]), .clk(clk), .out(R4[15:0]));
load_enable l5 (.in(data_in), .load(write), .write_bit(dc_write_out[5]), .clk(clk), .out(R5[15:0]));
load_enable l6 (.in(data_in), .load(write), .write_bit(dc_write_out[6]), .clk(clk), .out(R6[15:0]));
load_enable l7 (.in(data_in), .load(write), .write_bit(dc_write_out[7]), .clk(clk), .out(R7[15:0]));

//Read decoder
always_comb begin
    case (readnum)
    3'b000: data_out[15:0] = R0;
    3'b001: data_out[15:0] = R1;
    3'b010: data_out[15:0] = R2;
    3'b011: data_out[15:0] = R3;
    3'b100: data_out[15:0] = R4;
    3'b101: data_out[15:0] = R5;
    3'b110: data_out[15:0] = R6;
    3'b111: data_out[15:0] = R7;
    endcase
end


endmodule

module decoder(in, out);
input [2:0] in;
output reg [7:0] out;

always_comb begin
    case(in)
    3'b000: out[7:0] = 8'b00000001;
    3'b001: out[7:0] = 8'b00000010;
    3'b010: out[7:0] = 8'b00000100;
    3'b011: out[7:0] = 8'b00001000;
    3'b100: out[7:0] = 8'b00010000;
    3'b101: out[7:0] = 8'b00100000;
    3'b110: out[7:0] = 8'b01000000;
    3'b111: out[7:0] = 8'b10000000;
    default: out[7:0] = 8'bxxxxxxxx;
    endcase
end

endmodule

module load_enable(in, load, write_bit, clk, out);
input [15:0] in;
input load, clk, write_bit;
output reg [15:0] out;
wire [15:0] next_output;

assign next_output = load ? in: out;

always_ff @(posedge clk) begin
    if (write_bit === 1'b1)
    out <= next_output;
end

endmodule
