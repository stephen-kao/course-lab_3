module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    // System clock and Reset
    input  wire                   axis_clk,
    input  wire                   axis_rst_n,
    // AXI-Lite Protocal
    // write
    input  wire [pADDR_WIDTH-1:0] awaddr,
    input  wire                   awvalid,
    output wire                   awready,
    input  wire [pDATA_WIDTH-1:0] wdata,
    input  wire                   wvalid,
    output wire                   wready,
    // read
    input  wire [pADDR_WIDTH-1:0] araddr,
    input  wire                   arvalid,
    output wire                   arready,
    output wire [pDATA_WIDTH-1:0] rdata,  
    output wire                   rvalid,
    input  wire                   rready,
    // AXI-Stream Protocal
    // data in
    input  wire                   ss_tvalid, 
    input  wire [pDATA_WIDTH-1:0] ss_tdata, 
    input  wire                   ss_tlast, 
    output wire                   ss_tready, 
    // data out
    input  wire                   sm_tready, 
    output wire                   sm_tvalid, 
    output wire [pDATA_WIDTH-1:0] sm_tdata, 
    output wire                   sm_tlast, 
    // bram for tap RAM (store coefficient)
    output reg [3:0]              tap_WE,
    output reg                    tap_EN,
    output reg [pDATA_WIDTH-1:0]  tap_Di,
    output reg [pADDR_WIDTH-1:0]  tap_A,
    input  wire [pDATA_WIDTH-1:0] tap_Do,
    // bram for data RAM (store data?)
    output reg [3:0]              data_WE,
    output reg                    data_EN,
    output reg [pDATA_WIDTH-1:0]  data_Di,
    output reg [pADDR_WIDTH-1:0]  data_A,
    input  wire [pDATA_WIDTH-1:0] data_Do
);

//////////////////////////////////////////////////////////////////////////////////
//
// LOCAL PARAMETER DECLARATION
//
//////////////////////////////////////////////////////////////////////////////////
// ap signal
wire [pDATA_WIDTH-1:0] ap_signal; // ap_signal = {29{1'b0}, ap_idle, ap_done, ap_start}
reg                    ap_idle;
reg                    ap_done;
reg                    ap_start;
reg                    next_ap_idle;
reg                    next_ap_done;
reg                    next_ap_start;
// data length
reg  [pDATA_WIDTH-1:0] data_length;
// fsm
reg  [1:0]             state;
reg  [1:0]             next_state;
reg                    finish;
// counter
reg  [4:0]             next_cnt;
reg  [4:0]             cnt;
// coefficient buffer and data buffer
reg signed [pDATA_WIDTH-1:0] coeff [0:Tape_Num-1];
reg signed [pDATA_WIDTH-1:0] data  [0:Tape_Num-1];
integer i,j;
// fir design
wire [pDATA_WIDTH-1:0] temp_sum;
wire [pDATA_WIDTH-1:0] cur_sum;
reg  [pDATA_WIDTH-1:0] prev_sum;
reg  [pDATA_WIDTH-1:0] cur_coef;
reg  [pDATA_WIDTH-1:0] cur_data;
// fsm state
localparam IDLE = 'd0;
localparam LOAD = 'd1;
localparam WORK = 'd2;
localparam DONE = 'd3;
//////////////////////////////////////////////////////////////////////////////////
//
// AXI-Lite Handshake
//
//////////////////////////////////////////////////////////////////////////////////
assign awready = awvalid;
assign wready  = wvalid;
assign arready = arvalid;
assign rvalid  = rready;
//////////////////////////////////////////////////////////////////////////////////
//
// AXI-Stream Control
//
//////////////////////////////////////////////////////////////////////////////////
assign ss_tready = state==LOAD;
assign sm_tvalid = (state==LOAD && cnt==4'd11) | state==DONE;
assign sm_tdata  = prev_sum;
assign sm_tlast  = state==DONE;
//////////////////////////////////////////////////////////////////////////////////
//
// AP_SIGNAL
//
//////////////////////////////////////////////////////////////////////////////////
assign ap_signal = {{29{1'b0}}, ap_idle, ap_done, ap_start};
// ap_start
always @(*) begin
    if(state==IDLE && awaddr == 12'h00 && awvalid && wvalid && wdata == 32'h0000_0001) begin
        // write ap_start for short pulse, deassert when the engine is not IDLE
        next_ap_start = 1;
    end else begin
        next_ap_start = 0;
    end
end
// ap_idle
always @(*) begin
    if(next_ap_start) begin
        next_ap_idle = 0;
    end else if(state == DONE) begin
        next_ap_idle = 1;
    end else begin
        next_ap_idle = ap_idle;
    end
end
// ap_done
always @(*) begin
    if(state == DONE) begin
        next_ap_done = 1;
    end else begin
        next_ap_done = ap_done;
    end
end
always @(posedge axis_clk) begin
    if(~axis_rst_n) begin
        ap_idle  <= 1'b1;
        ap_done  <= 1'b0;
        ap_start <= 1'b0;
    end else begin
        ap_idle  <= next_ap_idle;
        ap_done  <= next_ap_done;
        ap_start <= next_ap_start;
    end
end
//////////////////////////////////////////////////////////////////////////////////
//
// Coefficient from AXI-Lite and store into coeff buffer and tap RAM
//
//////////////////////////////////////////////////////////////////////////////////
// When Read addr = 0x00 should read out ap_signal
// The others addr should read out the coefficient
assign rdata = (araddr == 12'h00) ? ap_signal : tap_Do;

// Write / Read tap RAM from AXI-Lite protocal
always @(*) begin
    if(awvalid) begin
        // write
        if((awaddr != 12'h00) && (awaddr != 12'h10)) begin
            tap_A  = awaddr - 12'h20;
            tap_EN = 1'b1;
            tap_WE = 4'b1111;
            tap_Di = wdata;
        end else begin
            tap_A  = 'd0;
            tap_EN = 'd0;
            tap_WE = 'd0;
            tap_Di = 'd0;    
        end
    end else if (arvalid) begin
        if(araddr != 12'h00 && araddr != 12'h10) begin
            tap_A  = araddr - 12'h20;
            tap_EN = 1'b1;
            tap_WE = 'd0;
            tap_Di = 'd0;
        end else begin
            tap_A  = 'd0;
            tap_EN = 'd0;
            tap_WE = 'd0;
            tap_Di = 'd0;
        end
    end else begin
        tap_A  = 'd0;
        tap_EN = 'd0;
        tap_WE = 'd0;
        tap_Di = 'd0;   
    end
end
//////////////////////////////////////////////////////////////////////////////////
//
// Data Length from AXI-Lite and store into data_length buffer
//
//////////////////////////////////////////////////////////////////////////////////
always @(posedge axis_clk) begin
    if(~axis_rst_n) begin
        data_length <= 'd0;
    end else if(awaddr == 12'h10) begin
        data_length <= wdata;
    end else begin
        data_length <= data_length;
    end
end
//////////////////////////////////////////////////////////////////////////////////
//
// Coefficient buffer and Data buffer
//
//////////////////////////////////////////////////////////////////////////////////
always @(posedge axis_clk) begin
    if(~axis_rst_n) begin
        for (i=0; i<Tape_Num; i=i+1) begin
            coeff[i] <= 0;
        end        
    end else if(awvalid && (awaddr!=12'h00 && awaddr!=12'h10) && cnt=='d0) begin
        for (i=0; i<Tape_Num-1; i=i+1) begin
            coeff[i] <= coeff[i+1];
        end
        coeff[Tape_Num-1] <= wdata;
    end else begin
        for (i=0; i<Tape_Num; i=i+1) begin
            coeff[i] <= coeff[i];
        end 
    end
end

always @(posedge axis_clk) begin
    if(~axis_rst_n) begin
        for (j=0; j<Tape_Num; j=j+1) begin
            data[j] <= 'd0;
        end 
    end else if(ss_tready) begin // data in, write into shift buffer
        for (j=0; j<Tape_Num-1; j=j+1) begin
            data[j] <= data[j+1];
        end
        data[Tape_Num-1] <= ss_tdata;
    end else begin
        for (j=0; j<Tape_Num; j=j+1) begin
            data[j] <= data[j];
        end 
    end
end

//////////////////////////////////////////////////////////////////////////////////
//
// FSM
//
//////////////////////////////////////////////////////////////////////////////////
always@(posedge axis_clk) begin
    if(~axis_rst_n) begin
        finish <= 1'b0;
    end else if(state==LOAD && ss_tlast) begin
        finish <= 1'b1;
    end else begin
        finish <= finish;
    end
end
always @(*) begin
    case(state)
        IDLE:    next_state = (ap_start) ? LOAD: IDLE;
        LOAD:    next_state = WORK;
        WORK:    next_state = (cnt == Tape_Num-1) ? (finish) ? DONE : LOAD : WORK;
        DONE:    next_state = DONE;
        default: next_state = IDLE;
    endcase
end

always @(posedge axis_clk) begin
    if(~axis_rst_n) begin
        state <= IDLE;
    end else begin
        state <= next_state;
    end
end
//////////////////////////////////////////////////////////////////////////////////
//
// Counter
//
//////////////////////////////////////////////////////////////////////////////////
always @(*) begin
    if(state==LOAD) begin
        next_cnt = 'd0;
    end else if(((state==IDLE)&&awvalid) | (state==WORK)) begin
        next_cnt = cnt + 1;
    end else begin
        next_cnt = 'd0;
    end
end

always @(posedge axis_clk) begin
    if(~axis_rst_n) begin
        cnt <= 'd0;
    end else begin
        cnt <= next_cnt;
    end
end
//////////////////////////////////////////////////////////////////////////////////
//
// FIR computation
//
//////////////////////////////////////////////////////////////////////////////////
assign temp_sum = cur_data * cur_coef;
assign cur_sum  = prev_sum + temp_sum;
always @(posedge axis_clk) begin
    if(~axis_rst_n) begin
        prev_sum <= 'd0;
        cur_coef <= 'd0;
        cur_data <= 'd0;
    end else if(state == WORK) begin
        prev_sum <= cur_sum;
        cur_coef <= coeff[cnt];
        cur_data <= data[cnt];
    end else begin
        prev_sum <= 'd0;
        cur_coef <= 'd0;
        cur_data <= 'd0;
    end
end

endmodule
