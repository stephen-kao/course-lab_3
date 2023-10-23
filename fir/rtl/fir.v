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
    output reg                    awready,
    input  wire [pDATA_WIDTH-1:0] wdata,
    input  wire                   wvalid,
    output reg                    wready,
    // read
    input  wire [pADDR_WIDTH-1:0] araddr,
    input  wire                   arvalid,
    output reg                    arready,
    output reg  [pDATA_WIDTH-1:0] rdata,  
    output reg                    rvalid,
    input  wire                   rready,
    // AXI-Stream Protocal
    // data in
    input  wire                   ss_tvalid, 
    input  wire [pDATA_WIDTH-1:0] ss_tdata, 
    input  wire                   ss_tlast, 
    output reg                    ss_tready, 
    // data out
    input  wire                   sm_tready, 
    output reg                    sm_tvalid, 
    output reg  [pDATA_WIDTH-1:0] sm_tdata, 
    output reg                    sm_tlast, 
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
reg                    ap_idle;
reg                    ap_done;
reg                    ap_start;
reg                    next_ap_idle;
reg                    next_ap_done;
reg                    next_ap_start;
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
// input buffer
wire [pADDR_WIDTH-1:0] awaddr_d;   
wire                   awvalid_d;  
wire [pDATA_WIDTH-1:0] wdata_d;   
wire                   wvalid_d;   
wire [pADDR_WIDTH-1:0] araddr_d;   
wire                   arvalid_d;  
wire                   rready_d;   
wire                   ss_tvalid_d;
wire [pDATA_WIDTH-1:0] ss_tdata_d; 
wire                   ss_tlast_d; 
wire                   sm_tready_d;
wire [pDATA_WIDTH-1:0] tap_Do_d;   
wire [pDATA_WIDTH-1:0] data_Do_d;  
// counter
reg [3:0] tap_addr_cnt;
reg [3:0] load_addr_cnt;
reg [9:0] data_num_cnt;
reg [3:0] flag;
reg [3:0] work_addr_cnt;
reg [3:0] next_tap_addr_cnt;
reg [3:0] next_load_addr_cnt;
reg [9:0] next_data_num_cnt;
reg [3:0] next_flag;
reg [3:0] next_work_addr_cnt;

input_buffer 
#(
    .pADDR_WIDTH    (pADDR_WIDTH),
    .pDATA_WIDTH    (pDATA_WIDTH)
) input_buf (
    .axis_clk       (axis_clk),
    //////////////////////////////////////////////////////////////////////////////////
    // INPUT
    //////////////////////////////////////////////////////////////////////////////////
    // AXI_Lite protocal
    .awaddr         (awaddr),
    .awvalid        (awvalid),
    .wdata          (wdata),
    .wvalid         (wvalid),
    .araddr         (araddr),
    .arvalid        (arvalid),
    .rready         (rready),
    // AXI_Stream protocal
    .ss_tvalid      (ss_tvalid),
    .ss_tdata       (ss_tdata),
    .ss_tlast       (ss_tlast),
    .sm_tready      (sm_tready),
    // tap sram
    .tap_Do         (tap_Do),
    // data sram
    .data_Do        (data_Do),
    //////////////////////////////////////////////////////////////////////////////////
    // OUTPUT
    //////////////////////////////////////////////////////////////////////////////////
    // AXI_Lite protocal
    .awaddr_d       (awaddr_d),
    .awvalid_d      (awvalid_d),
    .wdata_d        (wdata_d),
    .wvalid_d       (wvalid_d),
    .araddr_d       (araddr_d),
    .arvalid_d      (arvalid_d),
    .rready_d       (rready_d),
    // AXI_Stream protocal
    .ss_tvalid_d    (ss_tvalid_d),
    .ss_tdata_d     (ss_tdata_d),
    .ss_tlast_d     (ss_tlast_d),
    .sm_tready_d    (sm_tready_d),
    // tap sram
    .tap_Do_d       (tap_Do_d),
    // data sram
    .data_Do_d      (data_Do_d)
);

//////////////////////////////////////////////////////////////////////////////////
//
// AXI-Lite Handshake Control
//
//////////////////////////////////////////////////////////////////////////////////
wire next_awready;
wire next_wready;
wire next_arready;
wire next_rvalid;
// AXI-Lite handshake, 1 for handshake successfully
reg  write_handshake;
reg  read_handshake;
// rvalid trigger delay 
wire next_rvalid_trigger;
reg  rvalid_trigger;
reg  rvalid_trigger_d;

// AXI-Lite handshake whether successfully or not
always @(*) begin
    write_handshake = (awvalid_d && awready) && (wvalid_d && wready);
    read_handshake  = (arvalid_d && arready);
end

// AXI-Lite handshake control
// write
assign next_awready = awvalid_d;
assign next_wready  = wvalid_d;
// read
assign next_arready = arvalid_d;
assign next_rvalid  = (rvalid) ? 1'b0 : (~rvalid && ~arvalid_d) ? 1'b0 : (rvalid_trigger_d) ? 1'b1 : 1'b0;
assign next_rvalid_trigger  = ((tap_EN == 1'b1 && tap_WE == 'd0) && arvalid_d) | (state==DONE);
always @(posedge axis_clk) begin
    if(~axis_rst_n) begin
        rvalid_trigger    <= 'd0;
        rvalid_trigger_d  <= 'd0;
    end else if(~arvalid_d) begin
        rvalid_trigger    <= 'd0;
        rvalid_trigger_d  <= 'd0;
    end else begin
        rvalid_trigger    <= next_rvalid_trigger;
        rvalid_trigger_d  <= rvalid_trigger;
    end
end

always @(posedge axis_clk) begin
    if(~axis_rst_n) begin
        awready <= 'd0;
        wready  <= 'd0;
        arready <= 'd0;
        rvalid  <= 'd0;
    end else begin
        awready <= next_awready;
        wready  <= next_wready;
        arready <= next_arready;
        rvalid  <= next_rvalid;
    end
end
//////////////////////////////////////////////////////////////////////////////////
//
// AXI-Lite Protocal
// Addr = 12'h00: AP_Signal   (store in buffer)
// Addr = 12'h10: data_length (store in buffer)
// Addr = others: Coefficient (store in tap SRAM)
//
//////////////////////////////////////////////////////////////////////////////////
// ap_signal
reg  [pDATA_WIDTH-1:0] ap_signal;
wire [pDATA_WIDTH-1:0] next_ap_signal;
// data_length
reg  [pDATA_WIDTH-1:0] data_length;
wire [pDATA_WIDTH-1:0] next_data_length;
// tap SRAM 
reg  [pADDR_WIDTH-1:0] next_tap_A;
reg                    next_tap_EN;
reg              [3:0] next_tap_WE;
reg  [pDATA_WIDTH-1:0] next_tap_Di;


/////////////////////////////////////////
// AP Signal
/////////////////////////////////////////
assign next_ap_signal = {{29{1'b0}}, next_ap_idle, next_ap_done, next_ap_start};
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
        ap_idle   <= 1'b1;
        ap_done   <= 1'b0;
        ap_start  <= 1'b0;
        ap_signal <= {{29{1'b0}}, 1'b1, 1'b0, 1'b0};    // the three LSB order: {idle, done, start}
    end else begin
        ap_idle   <= next_ap_idle;
        ap_done   <= next_ap_done;
        ap_start  <= next_ap_start;
        ap_signal <= next_ap_signal;
    end
end

/////////////////////////////////////////
// Data Length buffer
/////////////////////////////////////////
assign next_data_length = (awaddr_d==12'h10 && write_handshake) ? wdata_d : data_length;
always @(posedge axis_clk) begin
    if(~axis_rst_n) begin
        data_length <= 'd0;
    end else begin
        data_length <= next_data_length;
    end
end

/////////////////////////////////////////
// coefficient
/////////////////////////////////////////
// When Read addr = 0x00 should read out ap_signal
// The others addr should read out the coefficient
always @(posedge axis_clk) begin
    if(~axis_rst_n) begin
        rdata <= 'd0;
    end else if(araddr_d==12'h00) begin
        rdata <= ap_signal;
    end else begin
        rdata <= tap_Do_d;
    end
end

// Write / Read tap RAM from AXI-Lite protocal
always @(*) begin
    if(write_handshake && awaddr_d >= 12'h20) begin
        // WRITE
        if((awaddr_d != 12'h00) && (awaddr_d != 12'h10)) begin
            // Neither ap-signal nor data_length, write for coefficient
            next_tap_A  = awaddr_d - 12'h20;
            next_tap_EN = 1'b1;
            next_tap_WE = 4'b1111;
            next_tap_Di = wdata_d;
        end else begin
            next_tap_A  = 'd0;
            next_tap_EN = 'd0;
            next_tap_WE = 'd0;
            next_tap_Di = 'd0;    
        end
    end else if (read_handshake && araddr_d >= 12'h20) begin
        // READ
        if((araddr_d != 12'h00) && (araddr_d != 12'h10)) begin
            // Neither ap-signal nor data_length, read for coefficient
            next_tap_A  = araddr_d - 12'h20;
            next_tap_EN = 1'b1;
            next_tap_WE = 'd0;
            next_tap_Di = 'd0;
        end else if(araddr_d == 12'h00) begin
            next_tap_A  = 'd0;
            next_tap_EN = 1'b1;
            next_tap_WE = 'd0;
            next_tap_Di = 'd0;
        end else begin
            next_tap_A  = 'd0;
            next_tap_EN = 'd0;
            next_tap_WE = 'd0;
            next_tap_Di = 'd0;
        end
    end else if(state==WORK) begin
        next_tap_A  = {tap_addr_cnt, {2'b0}};
        next_tap_EN = 1'b1;
        next_tap_WE = 'd0;
        next_tap_Di = 'd0;   
    end else begin
        next_tap_A  = 'd0;  
        next_tap_EN = 'd0;
        next_tap_WE = 'd0;
        next_tap_Di = 'd0; 
    end
end
always @(posedge axis_clk) begin
    if(~axis_rst_n) begin
        tap_A  <= 'd0;
        tap_EN <= 'd0;
        tap_WE <= 'd0;
        tap_Di <= 'd0;   
    end else begin
        tap_A  <= next_tap_A;
        tap_EN <= next_tap_EN;
        tap_WE <= next_tap_WE;
        tap_Di <= next_tap_Di;  
    end
end

//////////////////////////////////////////////////////////////////////////////////
//
// AXI-Stream Control
//
//////////////////////////////////////////////////////////////////////////////////
wire                   next_ss_tready;
wire                   next_sm_tvalid;
wire [pDATA_WIDTH-1:0] next_sm_tdata;
wire                   next_sm_tlast;

assign next_ss_tready = next_state==LOAD;
assign next_sm_tvalid = ((state==WORK) && (tap_addr_cnt==4'd2) && (data_num_cnt != 'd1)) | (state==DONE && tap_addr_cnt >= 4'd3);
assign next_sm_tdata  = prev_sum;
assign next_sm_tlast  = (state==DONE && tap_addr_cnt >= 4'd3);

always @(posedge axis_clk) begin
    if(~axis_rst_n) begin
        ss_tready <= 'd0;
        sm_tvalid <= 'd0;
        sm_tdata  <= 'd0;
        sm_tlast  <= 'd0;
    end else begin
        ss_tready <= next_ss_tready;
        sm_tvalid <= next_sm_tvalid;
        sm_tdata  <= next_sm_tdata;
        sm_tlast  <= next_sm_tlast;
    end
end
//////////////////////////////////////////////////////////////////////////////////
//
// Counter
//
//////////////////////////////////////////////////////////////////////////////////

// tap_addr_cnt
always @(*) begin
    if(state==LOAD | (state==WORK && next_state==DONE)) begin
        next_tap_addr_cnt = 'd0;
    end else if(((state==IDLE)&&awvalid) | (state==WORK) | (state==DONE)) begin
        next_tap_addr_cnt = tap_addr_cnt + 1;
    end else begin
        next_tap_addr_cnt = 'd0;
    end
end
// load_addr_cnt
always @(*) begin
    if(state==LOAD) begin
        if(load_addr_cnt==4'd10) begin
            next_load_addr_cnt = 'd0;
        end else begin
            next_load_addr_cnt = load_addr_cnt + 1;
        end
    end else begin
        next_load_addr_cnt = load_addr_cnt;
    end
end
// data_num_cnt
always @(*) begin
    if(state==LOAD) begin
        next_data_num_cnt = data_num_cnt + 1;
    end else begin
        next_data_num_cnt = data_num_cnt;
    end
end
// flag
always @(*) begin
    if(data_num_cnt<=4'd11) begin
        next_flag = 'd0;
    end else begin
        next_flag = load_addr_cnt + 1;
    end
end
// work_addr_cnt
always @(*) begin
    if(state==LOAD && next_state==WORK) begin
        if(load_addr_cnt==4'd10) begin
            next_work_addr_cnt = 4'd0;
        end else begin
            next_work_addr_cnt = load_addr_cnt + 1;
        end
    end else if(state==WORK) begin
        if(work_addr_cnt==4'd10) begin
            next_work_addr_cnt = 'd0;
        end else begin
            next_work_addr_cnt = work_addr_cnt + 1;
        end
    end else begin
        next_work_addr_cnt = 'd0;
    end
end

always @(posedge axis_clk) begin
    if(~axis_rst_n) begin
        tap_addr_cnt <= 'd0;
        load_addr_cnt <= 'd0;
        work_addr_cnt <= 'd0;
        flag <= 'd0;
        data_num_cnt <= 'd0;
    end else begin
        tap_addr_cnt <= next_tap_addr_cnt;
        load_addr_cnt <= next_load_addr_cnt;
        work_addr_cnt <= next_work_addr_cnt;
        flag <= next_flag;
        data_num_cnt <= next_data_num_cnt;
    end
end
//////////////////////////////////////////////////////////////////////////////////
//
// Data SRAM Control
//
//////////////////////////////////////////////////////////////////////////////////
reg  [pADDR_WIDTH-1:0] next_data_A;
reg                    next_data_EN;
reg              [3:0] next_data_WE;
reg  [pDATA_WIDTH-1:0] next_data_Di;

always @(*) begin
    case(state) 
        IDLE: begin
            next_data_A  = araddr_d - 12'h20;
            next_data_EN = 1'b1;
            next_data_WE = 4'b1111;
            next_data_Di = 'd0;
        end
        LOAD: begin
            next_data_A  = {load_addr_cnt, {2'b0}};   // which addr we should write the newest data
            next_data_EN = 1'b1;
            next_data_WE = 4'b1111;
            next_data_Di = ss_tdata_d;
        end
        WORK: begin
            next_data_A  = {work_addr_cnt, {2'b0}};   // which addr we need to compute
            next_data_EN = 1'b1;
            next_data_WE = 'd0;
            next_data_Di = 'd0;
        end 
        default: begin
            next_data_A  = 'd0;
            next_data_EN = 'd0;
            next_data_WE = 'd0;
            next_data_Di = 'd0;
        end
    endcase
end

always @(posedge axis_clk) begin
    if(~axis_rst_n) begin
        data_A <= 'd0;
        data_EN <= 'd0;
        data_WE <= 'd0;
        data_Di <= 'd0;
    end else begin
        data_A <= next_data_A;
        data_EN <= next_data_EN;
        data_WE <= next_data_WE;
        data_Di <= next_data_Di;
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
        WORK:    next_state = (tap_addr_cnt == Tape_Num-1) ? (finish) ? DONE : LOAD : WORK;
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

// //////////////////////////////////////////////////////////////////////////////////
// //
// // FIR computation
// //
// //////////////////////////////////////////////////////////////////////////////////
assign temp_sum = cur_data * cur_coef;
assign cur_sum  = prev_sum + temp_sum;
always @(posedge axis_clk) begin
    if(~axis_rst_n) begin
        prev_sum <= 'd0;
        cur_coef <= 'd0;
        cur_data <= 'd0;
    end else if(state == WORK && tap_addr_cnt==4'd2) begin
        prev_sum <= 'd0;
        cur_coef <= 'd0;
        cur_data <= 'd0;
    end else if(state==WORK | state==LOAD | state==DONE) begin
        prev_sum <= cur_sum;
        cur_coef <= tap_Do_d;
        cur_data <= data_Do_d;
    end else begin
        prev_sum <= 'd0;
        cur_coef <= 'd0;
        cur_data <= 'd0;
    end
end

endmodule

