module input_buffer
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32
)
(
    input axis_clk,
    //////////////////////////////////////////////////////////////////////////////////
    // INPUT
    //////////////////////////////////////////////////////////////////////////////////
    // AXI_Lite protocal
    input [pADDR_WIDTH-1:0] awaddr,         
    input awvalid,        
    input [pDATA_WIDTH-1:0] wdata,          
    input wvalid,         
    input [pADDR_WIDTH-1:0] araddr,         
    input arvalid,        
    input rready,         
    // AXI_Stream protocal
    input ss_tvalid,      
    input [pDATA_WIDTH-1:0] ss_tdata,       
    input ss_tlast,       
    input sm_tready,     
    // tap sram
    input [pDATA_WIDTH-1:0] tap_Do,         
    // data sram
    input [pDATA_WIDTH-1:0] data_Do,
    //////////////////////////////////////////////////////////////////////////////////
    // OUTPUT
    //////////////////////////////////////////////////////////////////////////////////
    // AXI_Lite protocal
    output reg [pADDR_WIDTH-1:0] awaddr_d,         
    output reg awvalid_d,        
    output reg [pDATA_WIDTH-1:0] wdata_d,          
    output reg wvalid_d,         
    output reg [pADDR_WIDTH-1:0] araddr_d,         
    output reg arvalid_d,        
    output reg rready_d,         
    // AXI_Stream protocal
    output reg ss_tvalid_d,      
    output reg [pDATA_WIDTH-1:0] ss_tdata_d,       
    output reg ss_tlast_d,       
    output reg sm_tready_d,     
    // tap sram
    output reg [pDATA_WIDTH-1:0] tap_Do_d,         
    // data sram
    output reg [pDATA_WIDTH-1:0] data_Do_d
);

always @(posedge axis_clk) begin
    // AXI_Lite protocal
    awaddr_d    <= awaddr;
    awvalid_d   <= awvalid;
    wdata_d     <= wdata;
    wvalid_d    <= wvalid;
    araddr_d    <= araddr;
    arvalid_d   <= arvalid;
    rready_d    <= rready;
    // AXI_Stream protocal
    ss_tvalid_d <= ss_tvalid;
    ss_tdata_d  <= ss_tdata;
    ss_tlast_d  <= ss_tlast;
    sm_tready_d <= sm_tready;
    // tap sram
    tap_Do_d    <= tap_Do;
    // data sram
    data_Do_d   <= data_Do;
end

endmodule