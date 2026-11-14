module if_stage (
    input  logic        clk,
    input  logic        reset,          // Asynchronous active-high reset
    input  logic        stall,          // Freeze PC and IF/EX register when high

    // Instruction fetched at 'address' (from instruction memory)
    input  logic [31:0] instr_f,

    // Redirect sources coming from stage 2 (EX)
    input  logic        and_out,        // Branch taken / JAL / JALR
    input  logic        trap_occurred,  // Exception or interrupt
    input  logic        is_mret_instr,  // MRET

    // Target addresses
    input  logic [31:0] pc_signed_offset, // Branch / JAL / JALR target
    input  logic [31:0] mepc,
    input  logic [31:0] mtvec,
    input  logic [31:0] trap_cause,

    // Stage 1 output: fetch PC (goes to instruction memory)
    output logic [31:0] address,

    // IF/EX pipeline register outputs (stage 2 inputs)
    output logic [31:0] instr_e,        // Instruction now in stage 2
    output logic [31:0] pc_e,           // PC of that instruction
    output logic [31:0] pc_plus4_e,     // pc_e + 4 (JAL/JALR link value)
    output logic        valid_e         // 0 = bubble (flushed slot)
);

    localparam logic [31:0] NOP      = 32'h0000_0013;  // addi x0,x0,0
    localparam logic [31:0] RESET_PC = 32'h8000_0000;

    // -------------------------------------------------------------------------
    // Trap target (Direct / Vectored)
    // -------------------------------------------------------------------------
    logic [31:0] mtvec_base;
    logic [31:0] trap_target;

    assign mtvec_base = {mtvec[31:2], 2'b00};

    always_comb begin
        if (mtvec[1:0] == 2'b01 && trap_cause[31])
            trap_target = mtvec_base + {trap_cause[29:0], 2'b00};
        else
            trap_target = mtvec_base;
    end

    // -------------------------------------------------------------------------
    // Next-PC mux  (priority: trap > mret > branch/jump > PC+4)
    // -------------------------------------------------------------------------
    logic [31:0] pc_next;
    logic        redirect;

    assign redirect = trap_occurred | is_mret_instr | and_out;

    always_comb begin
        if      (trap_occurred) pc_next = trap_target;
        else if (is_mret_instr) pc_next = mepc;
        else if (and_out)       pc_next = pc_signed_offset;
        else                    pc_next = address + 32'd4;
    end

    // -------------------------------------------------------------------------
    // STAGE 1: fetch PC register
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            address <= RESET_PC;
        else if (!stall)
            address <= pc_next;
    end

    // -------------------------------------------------------------------------
    // IF/EX pipeline register
    // On a redirect, the instruction fetched this cycle is on the wrong path,
    // so a NOP bubble is loaded instead (1-cycle penalty).
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            instr_e <= NOP;
            pc_e    <= RESET_PC;
            valid_e <= 1'b0;
        end else if (!stall) begin
            pc_e <= address;
            if (redirect) begin
                instr_e <= NOP;
                valid_e <= 1'b0;
            end else begin
                instr_e <= instr_f;
                valid_e <= 1'b1;
            end
        end
    end

    assign pc_plus4_e = pc_e + 32'd4;

endmodule