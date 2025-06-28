module axi_rr_arbiter #(
    parameter NUM_MASTERS = 2
)(
    input  wire                      clk,
    input  wire                      resetn,

    input  wire [NUM_MASTERS-1:0]    req,
    output reg  [NUM_MASTERS-1:0]    grant
);

    reg [$clog2(NUM_MASTERS)-1:0] current_priority;
    integer i;
    integer index;
    reg found;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            current_priority <= 0;
            grant <= 0;
        end else begin
            grant <= 0;
            found = 0;

            for (i = 0; i < NUM_MASTERS; i = i + 1) begin
                index = (current_priority + i) % NUM_MASTERS;
                if (!found && req[index]) begin
                    grant[index] <= 1'b1;
                    current_priority <= (index + 1) % NUM_MASTERS;
                    found = 1;
                end
            end
        end
    end

endmodule


module axi_address_decoder #(
    parameter ADDR_WIDTH = 32
)(
    input  wire [ADDR_WIDTH-1:0] addr,
    output wire [1:0]            slave_sel,
    output wire                  valid
);

    // Default: no match
    assign slave_sel = (addr >= 32'h00000000 && addr <= 32'h0000FFFF) ? 2'b01 :
                       (addr >= 32'h00010000 && addr <= 32'h0001FFFF) ? 2'b10 :
                       2'b00;

    assign valid = |slave_sel;  // High if any slave is selected

endmodule

module axi_write_address_channel #(
    parameter ADDR_WIDTH = 32,
    parameter ID_WIDTH   = 4,
    parameter LEN_WIDTH  = 8
)(
    input wire                     ACLK,
    input wire                     ARESETN,

    // Master 0
    input  wire                    AWVALID_0,
    input  wire [ADDR_WIDTH-1:0]   AWADDR_0,
    input  wire [ID_WIDTH-1:0]     AWID_0,
    input  wire [LEN_WIDTH-1:0]    AWLEN_0,
    input  wire [2:0]              AWSIZE_0,
    input  wire [1:0]              AWBURST_0,
    output wire                    AWREADY_0,

    // Master 1
    input  wire                    AWVALID_1,
    input  wire [ADDR_WIDTH-1:0]   AWADDR_1,
    input  wire [ID_WIDTH-1:0]     AWID_1,
    input  wire [LEN_WIDTH-1:0]    AWLEN_1,
    input  wire [2:0]              AWSIZE_1,
    input  wire [1:0]              AWBURST_1,
    output wire                    AWREADY_1,

    // Slave 0
    output reg                     M0_AWVALID,
    output reg  [ADDR_WIDTH-1:0]   M0_AWADDR,
    output reg  [ID_WIDTH-1:0]     M0_AWID,
    output reg  [LEN_WIDTH-1:0]    M0_AWLEN,
    output reg  [2:0]              M0_AWSIZE,
    output reg  [1:0]              M0_AWBURST,
    input  wire                    M0_AWREADY,

    // Slave 1
    output reg                     M1_AWVALID,
    output reg  [ADDR_WIDTH-1:0]   M1_AWADDR,
    output reg  [ID_WIDTH-1:0]     M1_AWID,
    output reg  [LEN_WIDTH-1:0]    M1_AWLEN,
    output reg  [2:0]              M1_AWSIZE,
    output reg  [1:0]              M1_AWBURST,
    input  wire                    M1_AWREADY
);

    wire [1:0] req = {AWVALID_1, AWVALID_0};
    wire [1:0] grant;

    reg [ADDR_WIDTH-1:0] selected_awaddr;
    reg [ID_WIDTH-1:0]   selected_awid;
    reg [LEN_WIDTH-1:0]  selected_awlen;
    reg [2:0]            selected_awsize;
    reg [1:0]            selected_awburst;

    wire [1:0] slave_sel;
    wire       sel_valid;

    // Instantiate Arbiter
    axi_rr_arbiter #(.NUM_MASTERS(2)) arb_aw (
        .clk    (ACLK),
        .resetn (ARESETN),
        .req    (req),
        .grant  (grant)
    );

    // Address selection based on granted master
    always @(*) begin
        if (grant[0]) begin
            selected_awaddr  = AWADDR_0;
            selected_awid    = AWID_0;
            selected_awlen   = AWLEN_0;
            selected_awsize  = AWSIZE_0;
            selected_awburst = AWBURST_0;
        end else if (grant[1]) begin
            selected_awaddr  = AWADDR_1;
            selected_awid    = AWID_1;
            selected_awlen   = AWLEN_1;
            selected_awsize  = AWSIZE_1;
            selected_awburst = AWBURST_1;
        end else begin
            selected_awaddr  = 32'h0;
            selected_awid    = 0;
            selected_awlen   = 0;
            selected_awsize  = 0;
            selected_awburst = 0;
        end
    end

    // Address decoder instance
    axi_address_decoder addr_dec_aw (
        .addr      (selected_awaddr),
        .slave_sel (slave_sel),
        .valid     (sel_valid)
    );

    // Route to selected slave
    always @(*) begin
        // Default outputs
        M0_AWVALID = 0;
        M0_AWADDR  = 0;
        M0_AWID    = 0;
        M0_AWLEN   = 0;
        M0_AWSIZE  = 0;
        M0_AWBURST = 0;

        M1_AWVALID = 0;
        M1_AWADDR  = 0;
        M1_AWID    = 0;
        M1_AWLEN   = 0;
        M1_AWSIZE  = 0;
        M1_AWBURST = 0;

        if (sel_valid) begin
            if (slave_sel == 2'b01) begin
                M0_AWVALID = |grant;
                M0_AWADDR  = selected_awaddr;
                M0_AWID    = selected_awid;
                M0_AWLEN   = selected_awlen;
                M0_AWSIZE  = selected_awsize;
                M0_AWBURST = selected_awburst;
            end else if (slave_sel == 2'b10) begin
                M1_AWVALID = |grant;
                M1_AWADDR  = selected_awaddr;
                M1_AWID    = selected_awid;
                M1_AWLEN   = selected_awlen;
                M1_AWSIZE  = selected_awsize;
                M1_AWBURST = selected_awburst;
            end
        end
    end

    // Backpressure to masters
    assign AWREADY_0 = grant[0] && ((slave_sel == 2'b01) ? M0_AWREADY :
                                    (slave_sel == 2'b10) ? M1_AWREADY : 1'b0);
    assign AWREADY_1 = grant[1] && ((slave_sel == 2'b01) ? M0_AWREADY :
                                    (slave_sel == 2'b10) ? M1_AWREADY : 1'b0);

endmodule


module axi_write_data_channel #(
    parameter DATA_WIDTH = 32,
    parameter STRB_WIDTH = DATA_WIDTH / 8
)(
    input  wire                  ACLK,
    input  wire                  ARESETN,

    // Master 0
    input  wire                  WVALID_0,
    input  wire [DATA_WIDTH-1:0] WDATA_0,
    input  wire [STRB_WIDTH-1:0] WSTRB_0,
    input  wire                  WLAST_0,
    output wire                  WREADY_0,

    // Master 1
    input  wire                  WVALID_1,
    input  wire [DATA_WIDTH-1:0] WDATA_1,
    input  wire [STRB_WIDTH-1:0] WSTRB_1,
    input  wire                  WLAST_1,
    output wire                  WREADY_1,

    // Slave 0
    output reg  [DATA_WIDTH-1:0] M0_WDATA,
    output reg  [STRB_WIDTH-1:0] M0_WSTRB,
    output reg                   M0_WVALID,
    output reg                   M0_WLAST,
    input  wire                  M0_WREADY,

    // Slave 1
    output reg  [DATA_WIDTH-1:0] M1_WDATA,
    output reg  [STRB_WIDTH-1:0] M1_WSTRB,
    output reg                   M1_WVALID,
    output reg                   M1_WLAST,
    input  wire                  M1_WREADY,

    // From AW Channel
    input  wire [1:0]            master_sel,   // One-hot encoded
    input  wire [1:0]            slave_sel,    // One-hot encoded
    input  wire                  valid_transfer
);

    // --------------------------
    // Backpressure to masters
    // --------------------------
    assign WREADY_0 = valid_transfer && master_sel[0] &&
                      ((slave_sel[0] && M0_WREADY) || (slave_sel[1] && M1_WREADY));

    assign WREADY_1 = valid_transfer && master_sel[1] &&
                      ((slave_sel[0] && M0_WREADY) || (slave_sel[1] && M1_WREADY));

    // --------------------------
    // Write data routing logic
    // --------------------------
    always @(*) begin
        // Defaults
        M0_WVALID = 0; M0_WDATA = 0; M0_WSTRB = 0; M0_WLAST = 0;
        M1_WVALID = 0; M1_WDATA = 0; M1_WSTRB = 0; M1_WLAST = 0;

        if (valid_transfer) begin
            // Master 0 active
            if (master_sel[0] && WVALID_0) begin
                if (slave_sel[0]) begin
                    M0_WVALID = 1;
                    M0_WDATA  = WDATA_0;
                    M0_WSTRB  = WSTRB_0;
                    M0_WLAST  = WLAST_0;
                end else if (slave_sel[1]) begin
                    M1_WVALID = 1;
                    M1_WDATA  = WDATA_0;
                    M1_WSTRB  = WSTRB_0;
                    M1_WLAST  = WLAST_0;
                end
            end
            // Master 1 active
            else if (master_sel[1] && WVALID_1) begin
                if (slave_sel[0]) begin
                    M0_WVALID = 1;
                    M0_WDATA  = WDATA_1;
                    M0_WSTRB  = WSTRB_1;
                    M0_WLAST  = WLAST_1;
                end else if (slave_sel[1]) begin
                    M1_WVALID = 1;
                    M1_WDATA  = WDATA_1;
                    M1_WSTRB  = WSTRB_1;
                    M1_WLAST  = WLAST_1;
                end
            end
        end
    end

endmodule


module axi_read_address_channel #(
    parameter ADDR_WIDTH = 32,
    parameter ID_WIDTH   = 4,
    parameter LEN_WIDTH  = 8
)(
    input wire                     ACLK,
    input wire                     ARESETN,

    // From Masters
    input  wire                    ARVALID_0,
    input  wire [ADDR_WIDTH-1:0]   ARADDR_0,
    input  wire [ID_WIDTH-1:0]     ARID_0,
    input  wire [LEN_WIDTH-1:0]    ARLEN_0,
    input  wire [2:0]              ARSIZE_0,
    input  wire [1:0]              ARBURST_0,
    output wire                    ARREADY_0,

    input  wire                    ARVALID_1,
    input  wire [ADDR_WIDTH-1:0]   ARADDR_1,
    input  wire [ID_WIDTH-1:0]     ARID_1,
    input  wire [LEN_WIDTH-1:0]    ARLEN_1,
    input  wire [2:0]              ARSIZE_1,
    input  wire [1:0]              ARBURST_1,
    output wire                    ARREADY_1,

    // To Slaves
    output reg  [ADDR_WIDTH-1:0]   M0_ARADDR,
    output reg  [ID_WIDTH-1:0]     M0_ARID,
    output reg  [LEN_WIDTH-1:0]    M0_ARLEN,
    output reg  [2:0]              M0_ARSIZE,
    output reg  [1:0]              M0_ARBURST,
    output reg                     M0_ARVALID,
    input  wire                    M0_ARREADY,

    output reg  [ADDR_WIDTH-1:0]   M1_ARADDR,
    output reg  [ID_WIDTH-1:0]     M1_ARID,
    output reg  [LEN_WIDTH-1:0]    M1_ARLEN,
    output reg  [2:0]              M1_ARSIZE,
    output reg  [1:0]              M1_ARBURST,
    output reg                     M1_ARVALID,
    input  wire                    M1_ARREADY
);

    wire [1:0] arvalid_bus = {ARVALID_1, ARVALID_0};
    wire [1:0] grant;

    reg  [ADDR_WIDTH-1:0] selected_araddr;
    reg  [ID_WIDTH-1:0]   selected_arid;
    reg  [LEN_WIDTH-1:0]  selected_arlen;
    reg  [2:0]            selected_arsize;
    reg  [1:0]            selected_arburst;

    wire [1:0] slave_sel;
    wire       sel_valid;

    // Arbiter
    axi_rr_arbiter #(.NUM_MASTERS(2)) arb_ar (
        .clk    (ACLK),
        .resetn (ARESETN),
        .req    (arvalid_bus),
        .grant  (grant)
    );

    // Grant-based selection
    always @(*) begin
        selected_araddr  = 0;
        selected_arid    = 0;
        selected_arlen   = 0;
        selected_arsize  = 0;
        selected_arburst = 0;

        if (grant[0]) begin
            selected_araddr  = ARADDR_0;
            selected_arid    = ARID_0;
            selected_arlen   = ARLEN_0;
            selected_arsize  = ARSIZE_0;
            selected_arburst = ARBURST_0;
        end else if (grant[1]) begin
            selected_araddr  = ARADDR_1;
            selected_arid    = ARID_1;
            selected_arlen   = ARLEN_1;
            selected_arsize  = ARSIZE_1;
            selected_arburst = ARBURST_1;
        end
    end

    // Address Decoder
    axi_address_decoder addr_dec_ar (
        .addr      (selected_araddr),
        .slave_sel (slave_sel),
        .valid     (sel_valid)
    );

    // Route signals to selected slave
    always @(*) begin
        M0_ARVALID = 0;
        M0_ARADDR  = 0;
        M0_ARID    = 0;
        M0_ARLEN   = 0;
        M0_ARSIZE  = 0;
        M0_ARBURST = 0;

        M1_ARVALID = 0;
        M1_ARADDR  = 0;
        M1_ARID    = 0;
        M1_ARLEN   = 0;
        M1_ARSIZE  = 0;
        M1_ARBURST = 0;

        if (sel_valid) begin
            if (slave_sel == 2'b01) begin
                M0_ARVALID = |grant;
                M0_ARADDR  = selected_araddr;
                M0_ARID    = selected_arid;
                M0_ARLEN   = selected_arlen;
                M0_ARSIZE  = selected_arsize;
                M0_ARBURST = selected_arburst;
            end else if (slave_sel == 2'b10) begin
                M1_ARVALID = |grant;
                M1_ARADDR  = selected_araddr;
                M1_ARID    = selected_arid;
                M1_ARLEN   = selected_arlen;
                M1_ARSIZE  = selected_arsize;
                M1_ARBURST = selected_arburst;
            end
        end
    end

    // Backpressure
    assign ARREADY_0 = grant[0] && sel_valid &&
                       ((slave_sel == 2'b01) ? M0_ARREADY :
                        (slave_sel == 2'b10) ? M1_ARREADY : 1'b0);

    assign ARREADY_1 = grant[1] && sel_valid &&
                       ((slave_sel == 2'b01) ? M0_ARREADY :
                        (slave_sel == 2'b10) ? M1_ARREADY : 1'b0);

endmodule

module axi_write_response_channel #(
    parameter ID_WIDTH = 4
)(
    input  wire                     ACLK,
    input  wire                     ARESETN,

    // From Slaves
    input  wire                     M0_BVALID,
    input  wire [1:0]               M0_BRESP,
    input  wire [ID_WIDTH-1:0]      M0_BID,
    output reg                      M0_BREADY,

    input  wire                     M1_BVALID,
    input  wire [1:0]               M1_BRESP,
    input  wire [ID_WIDTH-1:0]      M1_BID,
    output reg                      M1_BREADY,

    // To Masters
    output reg                      BVALID_0,
    output reg [1:0]                BRESP_0,
    output reg [ID_WIDTH-1:0]       BID_0,
    input  wire                     BREADY_0,

    output reg                      BVALID_1,
    output reg [1:0]                BRESP_1,
    output reg [ID_WIDTH-1:0]       BID_1,
    input  wire                     BREADY_1
);

    always @(*) begin
        // Defaults
        M0_BREADY = 0;
        M1_BREADY = 0;

        BVALID_0  = 0;
        BRESP_0   = 2'b00;
        BID_0     = 0;

        BVALID_1  = 0;
        BRESP_1   = 2'b00;
        BID_1     = 0;

        // Priority: M0 response first
        if (M0_BVALID) begin
            if (M0_BID[0] == 1'b0) begin
                BVALID_0  = 1;
                BRESP_0   = M0_BRESP;
                BID_0     = M0_BID;
                M0_BREADY = BREADY_0;
            end else begin
                BVALID_1  = 1;
                BRESP_1   = M0_BRESP;
                BID_1     = M0_BID;
                M0_BREADY = BREADY_1;
            end
        end
        else if (M1_BVALID) begin
            if (M1_BID[0] == 1'b0) begin
                BVALID_0  = 1;
                BRESP_0   = M1_BRESP;
                BID_0     = M1_BID;
                M1_BREADY = BREADY_0;
            end else begin
                BVALID_1  = 1;
                BRESP_1   = M1_BRESP;
                BID_1     = M1_BID;
                M1_BREADY = BREADY_1;
            end
        end
    end

endmodule


module axi_read_data_channel #(
    parameter DATA_WIDTH = 32,
    parameter ID_WIDTH   = 4
)(
    input  wire                      ACLK,
    input  wire                      ARESETN,

    // From Slave 0
    input  wire                      M0_RVALID,
    input  wire [DATA_WIDTH-1:0]     M0_RDATA,
    input  wire [1:0]                M0_RRESP,
    input  wire                      M0_RLAST,
    input  wire [ID_WIDTH-1:0]       M0_RID,
    output reg                       M0_RREADY,

    // From Slave 1
    input  wire                      M1_RVALID,
    input  wire [DATA_WIDTH-1:0]     M1_RDATA,
    input  wire [1:0]                M1_RRESP,
    input  wire                      M1_RLAST,
    input  wire [ID_WIDTH-1:0]       M1_RID,
    output reg                       M1_RREADY,

    // To Master 0
    output reg                       RVALID_0,
    output reg [DATA_WIDTH-1:0]      RDATA_0,
    output reg [1:0]                 RRESP_0,
    output reg                       RLAST_0,
    output reg [ID_WIDTH-1:0]        RID_0,
    input  wire                      RREADY_0,

    // To Master 1
    output reg                       RVALID_1,
    output reg [DATA_WIDTH-1:0]      RDATA_1,
    output reg [1:0]                 RRESP_1,
    output reg                       RLAST_1,
    output reg [ID_WIDTH-1:0]        RID_1,
    input  wire                      RREADY_1
);

    always @(*) begin
        // Default outputs
        M0_RREADY = 0;
        M1_RREADY = 0;

        RVALID_0  = 0;
        RDATA_0   = 0;
        RRESP_0   = 0;
        RLAST_0   = 0;
        RID_0     = 0;

        RVALID_1  = 0;
        RDATA_1   = 0;
        RRESP_1   = 0;
        RLAST_1   = 0;
        RID_1     = 0;

        // Priority: respond to slave 0 first if both valid
        if (M0_RVALID) begin
            if (M0_RID[0] == 1'b0) begin  // Master 0 (based on LSB of RID)
                RVALID_0  = 1;
                RDATA_0   = M0_RDATA;
                RRESP_0   = M0_RRESP;
                RLAST_0   = M0_RLAST;
                RID_0     = M0_RID;
                M0_RREADY = RREADY_0;
            end else begin               // Master 1
                RVALID_1  = 1;
                RDATA_1   = M0_RDATA;
                RRESP_1   = M0_RRESP;
                RLAST_1   = M0_RLAST;
                RID_1     = M0_RID;
                M0_RREADY = RREADY_1;
            end
        end else if (M1_RVALID) begin
            if (M1_RID[0] == 1'b0) begin  // Master 0
                RVALID_0  = 1;
                RDATA_0   = M1_RDATA;
                RRESP_0   = M1_RRESP;
                RLAST_0   = M1_RLAST;
                RID_0     = M1_RID;
                M1_RREADY = RREADY_0;
            end else begin               // Master 1
                RVALID_1  = 1;
                RDATA_1   = M1_RDATA;
                RRESP_1   = M1_RRESP;
                RLAST_1   = M1_RLAST;
                RID_1     = M1_RID;
                M1_RREADY = RREADY_1;
            end
        end
    end

endmodule
