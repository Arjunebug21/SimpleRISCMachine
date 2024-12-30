module lab7_top(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5);
input [3:0] KEY;
input [9:0] SW;
output [9:0] LEDR;
output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

`define MNONE 2'b00
`define MREAD 2'b01
`define MWRITE 2'b10

//Button instantiation
wire clk = ~KEY[0];
wire reset = ~KEY[1];

assign LEDR[9:8] = 2'b00;

//Between MEM and CPU
wire [15:0] write_data; //temporary wire for when I figure out where the output of datapath goes
wire N, V, Z; //Idek what to do with these rn
wire [1:0] mem_cmd; //Memory command output is 2 bits wide
wire [8:0] mem_addr; //Memory address output is 9 bits wide
wire [15:0] read_data; //Read data input is 16 bits wide and goes to instruction register
wire [15:0] dout;
wire write;

cpu CPU(.clk(clk), .reset(reset), 
        .out(write_data), .N(N), .V(V), .Z(Z), .mem_cmd(mem_cmd), //out is connected to datapath_out
        .mem_addr(mem_addr), .read_data(read_data));

reg enable;

always_comb begin //Enable logic
    if ((mem_cmd === `MREAD) && (mem_addr[8] === 1'b0)) begin
        enable = 1'b1;
    end else begin
        enable = 1'b0;
    end
end

assign read_data = enable ? dout : {16{1'bz}}; //Replace this with tri-state eventually
assign write = ((mem_cmd === `MWRITE) & (mem_addr[8] === 1'b0)) ? 1'b1 : 1'b0;

RAM MEM (.clk(clk), .read_address(mem_addr[7:0]), .write_address(mem_addr[7:0]), .write(write), .din(write_data), .dout(dout));

switch_control switch_mem(.mem_cmd(mem_cmd), .mem_addr(mem_addr), .read_data(read_data), .switch_values(SW[7:0]));

led_control led_mem(.clk(clk), .mem_addr(mem_addr), .mem_cmd(mem_cmd), .write_data(write_data[7:0]), .led_values(LEDR[7:0]));
endmodule

module clock_mod(CLOCK_50, clk_out);
input CLOCK_50;

assign clk_out = CLOCK_50;

endmodule

module RAM(clk,read_address,write_address,write,din,dout);
  parameter data_width = 16; 
  parameter addr_width = 8;
  parameter filename = "data.txt";

  input clk;
  input [addr_width-1:0] read_address, write_address;
  input write;
  input [data_width-1:0] din;
  output [data_width-1:0] dout;
  reg [data_width-1:0] dout;

  reg [data_width-1:0] mem [2**addr_width-1:0];

  initial $readmemb(filename, mem);

  always @ (posedge clk) begin
    if (write)
     mem[write_address] <= din;
    dout <= mem[read_address];
end 
endmodule

module switch_control(mem_cmd, mem_addr, read_data, switch_values);
input [1:0] mem_cmd;
input [8:0] mem_addr;
input [7:0] switch_values;
output [15:0] read_data;

reg enable;

always_comb begin
    if ((mem_cmd === `MREAD) && (mem_addr === 9'h140)) begin
        enable = 1'b1;
    end else begin
        enable = 1'b0;
    end
end

assign read_data[15:8] = enable ? 8'h00 : {8{1'bz}};
assign read_data[7:0] = enable ? switch_values : {8{1'bz}};

endmodule

module led_control(clk, mem_addr, mem_cmd, write_data, led_values);
input clk;
input [1:0] mem_cmd;
input [8:0] mem_addr;
input [7:0] write_data;
output [7:0] led_values;

reg load;

always_comb begin
    if ((mem_cmd === `MWRITE) && (mem_addr === 9'h100)) begin
        load = 1'b1;
    end else begin
        load = 1'b0;
    end
end

load_enable_led led_register(.in(write_data), .load(load), .clk(clk), .out(led_values));

endmodule

module load_enable_led(in, load, clk, out);
input [7:0] in;
input load, clk;
output reg [7:0] out;
wire [7:0] next_output;

assign next_output = load ? in: out;

always_ff @(posedge clk) begin
    out <= next_output;
end

endmodule