
// Code your design here
module SimpleCPU(clk, rst, data_fromRAM, wrEn, addr_toRAM, data_toRAM);

parameter SIZE = 10;

input clk, rst;
input wire [31:0] data_fromRAM;
output reg wrEn;
output reg [SIZE-1:0] addr_toRAM;
output reg [31:0] data_toRAM;

`define reset 0
`define fetch_0 1
`define fetch_1 2
`define decode_0 3
`define decode_1 4
`define decode_2 5
`define execute 6

reg [2:0] state_current, state_next;
reg [SIZE-1:0] pc_current, pc_next; // pc: program counter register
reg [31:0] iw_current, iw_next; // iw: instruction word register
reg [31:0] r1_current, r1_next; // general-purpose register 1
reg [31:0] r2_current, r2_next; // general-purpose register 2

always@(posedge clk) begin
	if(rst) begin
		state_current 	<= `reset;
		pc_current		<= 10'b0;
		iw_current 		<= 32'b0;
		r1_current 		<= 32'b0;
		r2_current 		<= 32'b0;
	end
	else begin
		state_current 	<= state_next;
		pc_current 		<= pc_next;
		iw_current 		<= iw_next;
		r1_current 		<= r1_next;
		r2_current 		<= r2_next;
	end
end

always@(*) begin
	state_next 	= state_current; // default assignment
	pc_next 	= pc_current; // default assignment
	iw_next 	= iw_current; // default assignment
	r1_next 	= r1_current; // default assignment
	r2_next 	= r2_current; // default assignment
	wrEn 		= 0; // default assignment
	addr_toRAM 	= 0; // default assignment
	data_toRAM 	= 0; // default assignment
	case(state_current)
		`reset: begin
			pc_next 	= 0;
			iw_next 	= 0;
			r1_next 	= 0;
			r2_next 	= 0;
			state_next	= `fetch_0;
		end
		`fetch_0: begin
			addr_toRAM 	= pc_current;
			state_next	= `fetch_1;
		end
		`fetch_1: begin
			iw_next 	= data_fromRAM;			
			state_next 	= `decode_0;
		end
		`decode_0: begin
			if(iw_current[31:28]=={3'b100, 1'b0}) begin //CP         
				state_next = `decode_1;
				end
			else if(iw_current[31:28]=={3'b100, 1'b1})begin//CPi 
        		r2_next=iw_current[13:0];
				state_next = `execute;
				end  
			else if(iw_current[31:28]=={3'b101, 1'b0})begin//CPI		  
                addr_toRAM = iw_current[13:0];
                state_next = `decode_1;
              	end
			else  begin //ADD//ADDi//NAND//NANDi//SRL//SRLi//LT//LTi//CPIi//BZJ//BZJi
				addr_toRAM = iw_current[27:14];
                state_next = `decode_1;
                end	
		end
		`decode_1: begin
			case(iw_current[31:28])          
              {3'b000, 1'b0}: begin//ADD
                r1_next= data_fromRAM;
                addr_toRAM= iw_current[13:0];
                state_next= `decode_2;
              end
              {3'b000, 1'b1}: begin//ADDi
                r1_next= data_fromRAM;
                r2_next= iw_current[13:0];
                state_next = `execute;
              end    
              {3'b001, 1'b0}: begin//NAND
                r1_next= data_fromRAM;
                addr_toRAM= iw_current[13:0];
                state_next = `decode_2;
              end
              {3'b001, 1'b1}: begin//NANDi    
                r1_next= data_fromRAM;
                r2_next= iw_current[13:0];
                state_next = `execute;
              end  
              {3'b010, 1'b0}: begin//SRL
                r1_next= data_fromRAM;
                addr_toRAM= iw_current[13:0];
                state_next = `decode_2;
              end
              {3'b010, 1'b1}: begin//SRLi
                r1_next= data_fromRAM;
                r2_next= iw_current[13:0];
                state_next = `execute;
              end  
              {3'b011, 1'b0}: begin//LT
                r1_next= data_fromRAM;
                addr_toRAM= iw_current[13:0];
                state_next = `decode_2;
              end
              {3'b011, 1'b1}: begin//LTi
                r1_next= data_fromRAM;
                r2_next= iw_current[13:0];
                state_next = `execute;
              end
              {3'b100, 1'b0}: begin// CP
				addr_toRAM= iw_current[13:0];
				state_next 	= `decode_2;
			  end 
              {3'b101, 1'b0}: begin//CPL
                r1_next= data_fromRAM;
                addr_toRAM=data_fromRAM;
                state_next = `decode_2;
              end
              {3'b101, 1'b1}: begin//CPLi
                r1_next= data_fromRAM;
                addr_toRAM= iw_current[13:0];
                state_next = `decode_2;
              end
              {3'b110, 1'b0}: begin//BZJ
                r1_next= data_fromRAM;
                addr_toRAM= iw_current[13:0];
                state_next = `decode_2;
              end
              {3'b110, 1'b1}: begin//BZJi
                r1_next= data_fromRAM;
                r2_next= iw_current[13:0];
                state_next = `execute;
              end
              {3'b111, 1'b0}: begin//MUL
                r1_next= data_fromRAM; 
                addr_toRAM= iw_current[13:0];
                state_next = `decode_2;
              end
              {3'b111, 1'b1}: begin//MULi
                r1_next= data_fromRAM;
                r2_next= iw_current[13:0];
                state_next = `execute;
              end
			endcase
		end
		`decode_2: begin
          if(iw_current[31:28]=={3'b101, 1'b0}) begin//CPI    
                r1_next= data_fromRAM;
                state_next = `execute;
              end
          else begin//ADD//NAND//SRL//LT//CP//CPIi//BZJ//MUL
           		r2_next= data_fromRAM;
                state_next = `execute;
          	  end  
		end
		`execute: begin
			casex(iw_current[31:28])
              {3'b000, 1'bx}: begin//ADD//ADDi
                wrEn= 1'b1;
                pc_next= pc_current+1;
                addr_toRAM= iw_current[27:14];
                data_toRAM= (r1_current+r2_current);
                state_next = `fetch_0;
              end
              {3'b001, 1'bx}: begin//NAND//NANDi
                wrEn = 1'b1;
                pc_next= pc_current+1;
                addr_toRAM= iw_current[27:14];
                data_toRAM= ~(r1_current & r2_current);
                state_next = `fetch_0;
              end
              {3'b010, 1'bx}: begin//SRL//SRLi
                wrEn= 1'b1;
                pc_next= pc_current+1;
                addr_toRAM= iw_current[27:14];
                data_toRAM= (r2_current < 32)? (r1_current >> r2_current) : (r1_current << (r2_current));
                state_next = `fetch_0;
              end  
              {3'b011, 1'bx}: begin//LT//LTi
                wrEn= 1'b1;
                pc_next= pc_current+1;
                addr_toRAM= iw_current[27:14];
                //data_toRAM= (r1_current < r2_current)? 1 : 0 // Doesnt work
                if (r1_current<r2_current)
                	data_toRAM=1;
                else
                	data_toRAM=0;
                state_next = `fetch_0;
              end
              {3'b100,1'bx}: begin//CP//CPi
				wrEn= 1'b1;
				pc_next= pc_current+1;
				addr_toRAM= iw_current[27:14];
				data_toRAM= r2_current;
				state_next 	= `fetch_0;
			  end
              {3'b101,1'b0}: begin//CPI
				wrEn= 1'b1;
				pc_next= pc_current+1;
				addr_toRAM= iw_current[27:14];
				data_toRAM= r1_current;
				state_next 	= `fetch_0;
			  end
              {3'b101,1'b1}: begin//CPIi
				wrEn= 1'b1;
				pc_next= pc_current+1;
				addr_toRAM= r1_current;
				data_toRAM= r2_current;
				state_next 	= `fetch_0;
			  end  
              {3'b110, 1'b0}: begin//BZC
                pc_next= (r2_current == 0)? r1_current : (pc_current+1);
                state_next = `fetch_0;
              end  
              {3'b110, 1'b1}: begin//BZCi
                pc_next= (r1_current + r2_current);
                state_next = `fetch_0;
              end
              {3'b111, 1'bx}: begin//MUL//MULi
                wrEn= 1'b1;
                pc_next= pc_current+1;
                addr_toRAM= iw_current[27:14];
                data_toRAM= (r1_current*r2_current);
                state_next = `fetch_0;
              end
			endcase
		end
	endcase
end

endmodule

module blram(clk, rst, i_we, i_addr, i_ram_data_in, o_ram_data_out);

parameter SIZE = 10, DEPTH = 1024;

input clk;
input rst;
input i_we;
input [SIZE-1:0] i_addr;
input [31:0] i_ram_data_in;
output reg [31:0] o_ram_data_out;

reg [31:0] memory[0:DEPTH-1];

always @(posedge clk) begin
  o_ram_data_out <= #1 memory[i_addr[SIZE-1:0]];
  if (i_we)
		memory[i_addr[SIZE-1:0]] <= #1 i_ram_data_in;
end 

initial begin
	memory[0] = 32'h20114045;
memory[1] = 32'h10114001;
memory[2] = 32'hb0118064;
memory[3] = 32'h190045;
memory[4] = 32'h8011c064;
memory[5] = 32'h7011c001;
memory[6] = 32'h10118001;
memory[7] = 32'hc0120047;
memory[8] = 32'h118045;
memory[9] = 32'ha0128046;
memory[10] = 32'h8012c046;
memory[11] = 32'h12c045;
memory[12] = 32'ha013004b;
memory[13] = 32'he013004a;
memory[14] = 32'h8012804c;
memory[15] = 32'h8013404b;
memory[16] = 32'h701343e9;
memory[17] = 32'hc012404d;
memory[18] = 32'h8019404c;
memory[19] = 32'hd0050013;
memory[20] = 32'h0;
memory[69] = 32'h1;
memory[70] = 32'h3e8;
memory[72] = 32'h2;
memory[73] = 32'hb;
memory[100] = 32'h6;
memory[101] = 32'h0;
memory[999] = 32'h1;

end 

endmodule
