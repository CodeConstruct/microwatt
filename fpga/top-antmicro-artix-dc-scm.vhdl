library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library unisim;
use unisim.vcomponents.all;

library work;
use work.wishbone_types.all;

entity toplevel is
    generic (
        MEMORY_SIZE        : integer  := 16384;
        RAM_INIT_FILE      : string   := "firmware.hex";
        RESET_LOW          : boolean  := true;
        CLK_FREQUENCY      : positive := 100000000;
        HAS_FPU            : boolean  := true;
        HAS_BTC            : boolean  := true;
        USE_LITEDRAM       : boolean  := false;
        NO_BRAM            : boolean  := false;
        DISABLE_FLATTEN_CORE : boolean := false;
        SCLK_STARTUPE2     : boolean := false;
        SPI_FLASH_OFFSET   : integer := 3145728;
        SPI_FLASH_DEF_CKDV : natural := 1;
        SPI_FLASH_DEF_QUAD : boolean := true;
        LOG_LENGTH         : natural := 512;
        USE_LITEETH        : boolean  := true;
        UART_IS_16550      : boolean  := false;
        HAS_UART1          : boolean  := true;
        USE_LITESDCARD     : boolean := true;
        HAS_LPC            : boolean := true;
        HAS_GPIO           : boolean := true;
        NGPIO              : natural := 32;
        NGPIOB             : natural := 32
        );
    port(
        ext_clk   : in  std_ulogic;

        -- UART0 signals:
        uart_main_tx : out std_ulogic;
        uart_main_rx : in  std_ulogic;

        -- TPM header used for gpio rework or litescope
        spi0_cs_n  : inout std_ulogic;
        spi0_mosi  : inout std_ulogic;
        spi0_miso  : inout std_ulogic;
        spi0_clk  : inout std_ulogic;
        tpm_pirq : inout std_ulogic;
        tpm_rst  : inout std_ulogic;
        tpm_gpio : inout std_ulogic;
        tpm_pp  : inout std_ulogic;

        -- LEDs
        usr_led : out std_ulogic_vector(2 downto 0);
        -- old names - these didn't seem to be working?
        --d11_led : out std_ulogic;
        --d12_led : out std_ulogic;
        --d13_led : out std_ulogic;

        -- GPIO
        sys_pwrbtn_n : inout std_ulogic;
        sys_pwrok : inout std_ulogic;
        hpm_stby_en : inout std_ulogic;
        fsi_clk : inout std_ulogic;
        fsi_dat : inout std_ulogic;
        bmc_fsi_in_ena : inout std_ulogic;

        -- I2C
        i2c_scl : inout std_ulogic_vector(12 downto 0);
        i2c_sda : inout std_ulogic_vector(12 downto 0);

        -- incorrect wiring, actually goes to i2c5_sda, edge A12
        -- pcie_bmc_clk_100_p : inout std_ulogic;

        -- LPC
        lpc_bmc_clk33 : in std_ulogic;
        lpc_frame_n : in std_ulogic;
        lpc_bmc_rst_n : in std_ulogic;
        lpc_lad : inout std_ulogic_vector(3 downto 0);

        -- SPI
        spi_flash_cs_n   : out std_ulogic;
        spi_flash_mosi   : inout std_ulogic;
        spi_flash_miso   : inout std_ulogic;
        spi_flash_wp_n   : inout std_ulogic;
        spi_flash_hold_n : inout std_ulogic;

        -- Ethernet
        eth_clocks_tx    : out std_ulogic;
        eth_clocks_rx    : in std_ulogic;
        eth_rst_n        : out std_ulogic;
        eth_int_n        : in std_ulogic;
        eth_mdio         : inout std_ulogic;
        eth_mdc          : out std_ulogic;
        eth_rx_ctl       : in std_ulogic;
        eth_rx_data      : in std_ulogic_vector(3 downto 0);
        eth_tx_ctl       : out std_ulogic;
        eth_tx_data      : out std_ulogic_vector(3 downto 0);

        -- SD card
        -- note that only 4 bits are used by litesdcard
        sdcard_data   : inout std_ulogic_vector(7 downto 0);
        sdcard_cmd    : inout std_ulogic;
        sdcard_clk    : out   std_ulogic;
        -- rst_n is disabled until host sets a register to enable,
        -- see eMMC chip datasheet.
        -- sdcard_rstn is unused by litesdcard
        sdcard_rstn   : out   std_ulogic;

        -- DRAM wires
        ddram_a       : out std_ulogic_vector(14 downto 0);
        ddram_ba      : out std_ulogic_vector(2 downto 0);
        ddram_ras_n   : out std_ulogic;
        ddram_cas_n   : out std_ulogic;
        ddram_we_n    : out std_ulogic;
        ddram_dm      : out std_ulogic_vector(1 downto 0);
        ddram_dq      : inout std_ulogic_vector(15 downto 0);
        ddram_dqs_p   : inout std_ulogic_vector(1 downto 0);
        ddram_dqs_n   : inout std_ulogic_vector(1 downto 0);
        ddram_clk_p   : out std_ulogic;
        ddram_clk_n   : out std_ulogic;
        ddram_cke     : out std_ulogic;
        ddram_odt     : out std_ulogic;
        ddram_reset_n : out std_ulogic
        );
end entity toplevel;

architecture behaviour of toplevel is

    -- Reset signals:
    signal soc_rst : std_ulogic;
    signal pll_rst : std_ulogic;
    signal ext_rst_n : std_ulogic;
    signal sw_rst  : std_ulogic;
    signal do_rst_n: std_ulogic;

    -- Internal clock signals:
    signal system_clk        : std_ulogic;
    signal system_clk_locked : std_ulogic;

    -- External IOs from the SoC
    signal wb_ext_io_in        : wb_io_master_out;
    signal wb_ext_io_out       : wb_io_slave_out;
    signal wb_ext_is_dram_csr  : std_ulogic;
    signal wb_ext_is_dram_init : std_ulogic;
    signal wb_ext_is_eth       : std_ulogic;
    signal wb_ext_is_sdcard    : std_ulogic;

    -- DRAM main data wishbone connection
    signal wb_dram_in          : wishbone_master_out;
    signal wb_dram_out         : wishbone_slave_out;

    -- DRAM control wishbone connection
    signal wb_dram_ctrl_out    : wb_io_slave_out := wb_io_slave_out_init;

    -- LiteEth connection
    signal ext_irq_eth         : std_ulogic;
    signal wb_eth_out          : wb_io_slave_out := wb_io_slave_out_init;

    -- LiteSDCard connection
    signal ext_irq_sdcard      : std_ulogic := '0';
    signal wb_sdcard_out       : wb_io_slave_out := wb_io_slave_out_init;
    signal wb_sddma_out        : wb_io_master_out := wb_io_master_out_init;
    signal wb_sddma_in         : wb_io_slave_out;
    signal wb_sddma_nr         : wb_io_master_out;
    signal wb_sddma_ir         : wb_io_slave_out;
    -- for conversion from non-pipelined wishbone to pipelined
    signal wb_sddma_stb_sent   : std_ulogic;

    -- Control/status
    signal core_alt_reset : std_ulogic;

    -- Status LED
    signal led0_b_pwm : std_ulogic;
    signal led0_r_pwm : std_ulogic;
    signal led0_g_pwm : std_ulogic;

    -- Dumb PWM for the LEDs, those RGB LEDs are too bright otherwise
    signal pwm_counter  : std_ulogic_vector(8 downto 0);

    -- SPI flash
    signal spi_sck     : std_ulogic;
    signal spi_cs_n    : std_ulogic;
    signal spi_sdat_o  : std_ulogic_vector(3 downto 0);
    signal spi_sdat_oe : std_ulogic_vector(3 downto 0);
    signal spi_sdat_i  : std_ulogic_vector(3 downto 0);

    -- GPIO
    signal gpio_in     : std_ulogic_vector(NGPIO - 1 downto 0);
    signal gpio_out    : std_ulogic_vector(NGPIO - 1 downto 0);
    signal gpio_dir    : std_ulogic_vector(NGPIO - 1 downto 0);

    signal gpiob_in    : std_ulogic_vector(NGPIOB - 1 downto 0);
    signal gpiob_out   : std_ulogic_vector(NGPIOB - 1 downto 0);
    signal gpiob_dir   : std_ulogic_vector(NGPIOB - 1 downto 0);

    -- ddram clock signals as vectors
    signal ddram_clk_p_vec : std_logic_vector(0 downto 0);
    signal ddram_clk_n_vec : std_logic_vector(0 downto 0);

    -- LPC
    signal lpc_data_o        : std_ulogic_vector(3 downto 0);
    -- XXX matt - why reg?
    signal lpc_data_o_reg    : std_ulogic_vector(3 downto 0);
    signal lpc_data_oe       : std_ulogic;
    signal lpc_data_i        : std_ulogic_vector(3 downto 0);
    signal lpc_irq_o         : std_ulogic;
    signal lpc_irq_oe        : std_ulogic;
    signal lpc_irq_i         : std_ulogic;
    --signal lpc_frame_n       : std_ulogic;
    signal lpc_reset_n       : std_ulogic;
    signal lpc_clock         : std_ulogic;

    -- Fixup various memory sizes based on generics
    function get_bram_size return natural is
    begin
        if USE_LITEDRAM and NO_BRAM then
            return 0;
        else
            return MEMORY_SIZE;
        end if;
    end function;

    function get_payload_size return natural is
    begin
        if USE_LITEDRAM and NO_BRAM then
            return MEMORY_SIZE;
        else
            return 0;
        end if;
    end function;
    
    constant BRAM_SIZE    : natural := get_bram_size;
    constant PAYLOAD_SIZE : natural := get_payload_size;
begin

    -- Main SoC
    soc0: entity work.soc
        generic map(
            MEMORY_SIZE        => BRAM_SIZE,
            RAM_INIT_FILE      => RAM_INIT_FILE,
            SIM                => false,
            CLK_FREQ           => CLK_FREQUENCY,
            HAS_FPU            => HAS_FPU,
            HAS_BTC            => HAS_BTC,
            HAS_DRAM           => USE_LITEDRAM,
            DRAM_SIZE          => 512 * 1024 * 1024,
            DRAM_INIT_SIZE     => PAYLOAD_SIZE,
            DISABLE_FLATTEN_CORE => DISABLE_FLATTEN_CORE,
            HAS_SPI_FLASH      => true,
            SPI_FLASH_DLINES   => 4,
            SPI_FLASH_OFFSET   => SPI_FLASH_OFFSET,
            SPI_FLASH_DEF_CKDV => SPI_FLASH_DEF_CKDV,
            SPI_FLASH_DEF_QUAD => SPI_FLASH_DEF_QUAD,
            LOG_LENGTH         => LOG_LENGTH,
            HAS_LITEETH        => USE_LITEETH,
            UART0_IS_16550     => UART_IS_16550,
            HAS_UART1          => HAS_UART1,
            HAS_SD_CARD        => USE_LITESDCARD,
            HAS_LPC            => HAS_LPC,
            HAS_GPIO           => HAS_GPIO,
            NGPIO              => NGPIO,
            HAS_GPIOB          => true,
            NGPIOB             => NGPIOB
            )
        port map (
            -- System signals
            system_clk        => system_clk,
            rst               => soc_rst,
            sw_soc_reset      => sw_rst,

            -- UART signals
            uart0_txd         => uart_main_tx,
            uart0_rxd         => uart_main_rx,

	    -- UART1 signals
	    --uart1_txd         => uart_pmod_tx,
	    --uart1_rxd         => uart_pmod_rx,

            -- SPI signals
            spi_flash_sck     => spi_sck,
            spi_flash_cs_n    => spi_cs_n,
            spi_flash_sdat_o  => spi_sdat_o,
            spi_flash_sdat_oe => spi_sdat_oe,
            spi_flash_sdat_i  => spi_sdat_i,

            -- GPIO signals
            gpio_in           => gpio_in,
            gpio_out          => gpio_out,
            gpio_dir          => gpio_dir,

            gpiob_in           => gpiob_in,
            gpiob_out          => gpiob_out,
            gpiob_dir          => gpiob_dir,

            -- LPC
            lpc_data_o        => lpc_data_o,
            lpc_data_oe       => lpc_data_oe,
            lpc_data_i        => lpc_data_i,
            lpc_frame_n       => lpc_frame_n,
            lpc_reset_n       => lpc_bmc_rst_n,
            lpc_clock         => lpc_bmc_clk33,
            lpc_irq_o         => lpc_irq_o,
            lpc_irq_oe        => lpc_irq_oe,
            lpc_irq_i         => lpc_irq_i,

            -- External interrupts
            ext_irq_eth       => ext_irq_eth,
            ext_irq_sdcard    => ext_irq_sdcard,

            -- DRAM wishbone
            wb_dram_in           => wb_dram_in,
            wb_dram_out          => wb_dram_out,

            -- IO wishbone
            wb_ext_io_in         => wb_ext_io_in,
            wb_ext_io_out        => wb_ext_io_out,
            wb_ext_is_dram_csr   => wb_ext_is_dram_csr,
            wb_ext_is_dram_init  => wb_ext_is_dram_init,
            wb_ext_is_eth        => wb_ext_is_eth,
            wb_ext_is_sdcard     => wb_ext_is_sdcard,

            -- DMA wishbone
            wishbone_dma_in      => wb_sddma_in,
            wishbone_dma_out     => wb_sddma_out,

            alt_reset            => core_alt_reset
            );

    -- LPC
    lpc_clock <= lpc_bmc_clk33;
    lpc_data_i <= lpc_lad;
    lpc_data_o_reg <= lpc_data_o;
    lpc_lad <= lpc_data_o_reg when lpc_data_oe = '1' and ext_rst_n = '1' else "ZZZZ";
    lpc_reset_n <= lpc_bmc_rst_n;
    --lpc_frame_n <= lpc_frame_n;

    -- not exposed by lpc gateware, is just part of soc
    -- would be bmc_serirq aka ESPI_ALERT_N
    --bmc_serirq <= lpc_irq_o  when lpc_irq_oe  = '1' and ext_rst_n = '1' else 'Z';
    --lpc_irq_i <= bmc_serirq
    lpc_irq_i <= '0';

    -- SPI Flash
    --
    -- Note: Unlike many other boards, the SPI flash on the Arty has
    -- an actual pin to generate the clock and doesn't require to use
    -- the STARTUPE2 primitive.
    --
    spi_flash_cs_n   <= spi_cs_n;
    spi_flash_mosi   <= spi_sdat_o(0) when spi_sdat_oe(0) = '1' else 'Z';
    spi_flash_miso   <= spi_sdat_o(1) when spi_sdat_oe(1) = '1' else 'Z';
    spi_flash_wp_n   <= spi_sdat_o(2) when spi_sdat_oe(2) = '1' else 'Z';
    spi_flash_hold_n <= spi_sdat_o(3) when spi_sdat_oe(3) = '1' else 'Z';
    spi_sdat_i(0)    <= spi_flash_mosi;
    spi_sdat_i(1)    <= spi_flash_miso;
    spi_sdat_i(2)    <= spi_flash_wp_n;
    spi_sdat_i(3)    <= spi_flash_hold_n;

    STARTUPE2_INST: STARTUPE2
        port map (
            CLK => '0',
            GSR => '0',
            GTS => '0',
            KEYCLEARB => '0',
            PACK => '0',
            USRCCLKO => spi_sck,
            USRCCLKTS => '0',
            USRDONEO => '1',
            USRDONETS => '0'
            );

    nodram: if not USE_LITEDRAM generate
        signal ddram_clk_dummy : std_ulogic;
    begin
        reset_controller: entity work.soc_reset
            generic map(
                RESET_LOW => RESET_LOW
                )
            port map(
                ext_clk => ext_clk,
                pll_clk => system_clk,
                pll_locked_in => system_clk_locked,
                ext_rst_in => do_rst_n,
                pll_rst_out => pll_rst,
                rst_out => soc_rst
                );

        clkgen: entity work.clock_generator
            generic map(
                CLK_INPUT_HZ => 100000000,
                CLK_OUTPUT_HZ => CLK_FREQUENCY
                )
            port map(
                ext_clk => ext_clk,
                pll_rst_in => pll_rst,
                pll_clk_out => system_clk,
                pll_locked_out => system_clk_locked
                );

	core_alt_reset <= '0';

        --d11_led <= '0';
        --d12_led <= soc_rst;
        --d13_led <= system_clk;

        -- Vivado barfs on those differential signals if left
        -- unconnected. So instanciate a diff. buffer and feed
        -- it a constant '0'.
        dummy_dram_clk: OBUFDS
            port map (
                O => ddram_clk_p,
                OB => ddram_clk_n,
                I => ddram_clk_dummy
                );
        ddram_clk_dummy <= '0';

    end generate;

    has_dram: if USE_LITEDRAM generate
        signal dram_init_done  : std_ulogic;
        signal dram_init_error : std_ulogic;
        signal dram_sys_rst    : std_ulogic;
        signal rst_gen_rst     : std_ulogic;
    begin

        -- Eventually dig out the frequency from the generator
        -- but for now, assert it's 100Mhz
        assert CLK_FREQUENCY = 100000000;

        reset_controller: entity work.soc_reset
            generic map(
                RESET_LOW => RESET_LOW,
                PLL_RESET_BITS => 18,
                SOC_RESET_BITS => 1
                )
            port map(
                ext_clk => ext_clk,
                pll_clk => system_clk,
                pll_locked_in => '1',
                ext_rst_in => do_rst_n,
                pll_rst_out => pll_rst,
                rst_out => open
                );

        -- Generate SoC reset
        soc_rst_gen: process(system_clk)
        begin
            if do_rst_n = '0' then
                soc_rst <= '1';
            elsif rising_edge(system_clk) then
                soc_rst <= dram_sys_rst or not system_clk_locked;
            end if;
        end process;

	ddram_clk_p_vec <= (others => ddram_clk_p);
	ddram_clk_n_vec <= (others => ddram_clk_n);

        dram: entity work.litedram_wrapper
            generic map(
                DRAM_ABITS => 25,
                DRAM_ALINES => 15,
                DRAM_DLINES => 16,
                DRAM_CKLINES => 1,
                DRAM_PORT_WIDTH => 128,
                PAYLOAD_FILE => RAM_INIT_FILE,
                PAYLOAD_SIZE => PAYLOAD_SIZE
                )
            port map(
                clk_in          => ext_clk,
                rst             => pll_rst,
                system_clk      => system_clk,
                system_reset    => dram_sys_rst,
                core_alt_reset  => core_alt_reset,
		pll_locked	=> system_clk_locked,

                wb_in           => wb_dram_in,
                wb_out          => wb_dram_out,
                wb_ctrl_in      => wb_ext_io_in,
                wb_ctrl_out     => wb_dram_ctrl_out,
                wb_ctrl_is_csr  => wb_ext_is_dram_csr,
                wb_ctrl_is_init => wb_ext_is_dram_init,

                init_done       => dram_init_done,
                init_error      => dram_init_error,

                ddram_a         => ddram_a,
                ddram_ba        => ddram_ba,
                ddram_ras_n     => ddram_ras_n,
                ddram_cas_n     => ddram_cas_n,
                ddram_we_n      => ddram_we_n,
                ddram_cs_n      => open,
                ddram_dm        => ddram_dm,
                ddram_dq        => ddram_dq,
                ddram_dqs_p     => ddram_dqs_p,
                ddram_dqs_n     => ddram_dqs_n,
                ddram_clk_p     => ddram_clk_p_vec,
                ddram_clk_n     => ddram_clk_n_vec,
                ddram_cke       => ddram_cke,
                ddram_odt       => ddram_odt,
                ddram_reset_n   => ddram_reset_n
                );

        --d11_led <= not dram_init_done;
        --d12_led <= soc_rst;
        --d13_led <= dram_init_error;

    end generate;

    has_liteeth : if USE_LITEETH generate

        component liteeth_core port (
            sys_clock           : in std_ulogic;
            sys_reset           : in std_ulogic;
            rgmii_eth_clocks_tx : out std_ulogic;
            rgmii_eth_clocks_rx : in std_ulogic;
            rgmii_eth_rst_n     : out std_ulogic;
            rgmii_eth_int_n     : in std_ulogic;
            rgmii_eth_mdio      : inout std_ulogic;
            rgmii_eth_mdc       : out std_ulogic;
            rgmii_eth_rx_ctl    : in std_ulogic;
            rgmii_eth_rx_data   : in std_ulogic_vector(3 downto 0);
            rgmii_eth_tx_ctl    : out std_ulogic;
            rgmii_eth_tx_data   : out std_ulogic_vector(3 downto 0);
            wishbone_adr        : in std_ulogic_vector(29 downto 0);
            wishbone_dat_w      : in std_ulogic_vector(31 downto 0);
            wishbone_dat_r      : out std_ulogic_vector(31 downto 0);
            wishbone_sel        : in std_ulogic_vector(3 downto 0);
            wishbone_cyc        : in std_ulogic;
            wishbone_stb        : in std_ulogic;
            wishbone_ack        : out std_ulogic;
            wishbone_we         : in std_ulogic;
            wishbone_cti        : in std_ulogic_vector(2 downto 0);
            wishbone_bte        : in std_ulogic_vector(1 downto 0);
            wishbone_err        : out std_ulogic;
            interrupt           : out std_ulogic
            );
        end component;

        signal wb_eth_cyc     : std_ulogic;
        signal wb_eth_adr     : std_ulogic_vector(29 downto 0);

    begin
        liteeth :  liteeth_core
            port map(
                sys_clock           => system_clk,
                sys_reset           => soc_rst,
                rgmii_eth_clocks_tx => eth_clocks_tx,
                rgmii_eth_clocks_rx => eth_clocks_rx,
                rgmii_eth_rst_n     => eth_rst_n,
                rgmii_eth_int_n     => eth_int_n,
                rgmii_eth_mdio      => eth_mdio,
                rgmii_eth_mdc       => eth_mdc,
                rgmii_eth_rx_ctl    => eth_rx_ctl,
                rgmii_eth_rx_data   => eth_rx_data,
                rgmii_eth_tx_ctl    => eth_tx_ctl,
                rgmii_eth_tx_data   => eth_tx_data,
                wishbone_adr        => wb_eth_adr,
                wishbone_dat_w      => wb_ext_io_in.dat,
                wishbone_dat_r      => wb_eth_out.dat,
                wishbone_sel        => wb_ext_io_in.sel,
                wishbone_cyc        => wb_eth_cyc,
                wishbone_stb        => wb_ext_io_in.stb,
                wishbone_ack        => wb_eth_out.ack,
                wishbone_we         => wb_ext_io_in.we,
                wishbone_cti        => "000",
                wishbone_bte        => "00",
                wishbone_err        => open,
                interrupt           => ext_irq_eth
                );

        -- Gate cyc with "chip select" from soc
        wb_eth_cyc <= wb_ext_io_in.cyc and wb_ext_is_eth;

        -- Remove top address bits as liteeth decoder doesn't know about them
        wb_eth_adr <= x"000" & "000" & wb_ext_io_in.adr(14 downto 0);

        -- LiteETH isn't pipelined
        wb_eth_out.stall <= not wb_eth_out.ack;

    end generate;

    no_liteeth : if not USE_LITEETH generate
        ext_irq_eth    <= '0';
    end generate;

    has_sdcard : if USE_LITESDCARD generate
        component litesdcard_core port (
            clk           : in    std_ulogic;
            rst           : in    std_ulogic;
            -- wishbone for accessing control registers
            wb_ctrl_adr   : in    std_ulogic_vector(29 downto 0);
            wb_ctrl_dat_w : in    std_ulogic_vector(31 downto 0);
            wb_ctrl_dat_r : out   std_ulogic_vector(31 downto 0);
            wb_ctrl_sel   : in    std_ulogic_vector(3 downto 0);
            wb_ctrl_cyc   : in    std_ulogic;
            wb_ctrl_stb   : in    std_ulogic;
            wb_ctrl_ack   : out   std_ulogic;
            wb_ctrl_we    : in    std_ulogic;
            wb_ctrl_cti   : in    std_ulogic_vector(2 downto 0);
            wb_ctrl_bte   : in    std_ulogic_vector(1 downto 0);
            wb_ctrl_err   : out   std_ulogic;
            -- wishbone for SD card core to use for DMA
            wb_dma_adr    : out   std_ulogic_vector(29 downto 0);
            wb_dma_dat_w  : out   std_ulogic_vector(31 downto 0);
            wb_dma_dat_r  : in    std_ulogic_vector(31 downto 0);
            wb_dma_sel    : out   std_ulogic_vector(3 downto 0);
            wb_dma_cyc    : out   std_ulogic;
            wb_dma_stb    : out   std_ulogic;
            wb_dma_ack    : in    std_ulogic;
            wb_dma_we     : out   std_ulogic;
            wb_dma_cti    : out   std_ulogic_vector(2 downto 0);
            wb_dma_bte    : out   std_ulogic_vector(1 downto 0);
            wb_dma_err    : in    std_ulogic;
            -- connections to SD card
            sdcard_data   : inout std_ulogic_vector(3 downto 0);
            sdcard_cmd    : inout std_ulogic;
            sdcard_clk    : out   std_ulogic;
            sdcard_cd     : in    std_ulogic;
            irq           : out   std_ulogic
            );
        end component;

        signal wb_sdcard_cyc : std_ulogic;
        signal wb_sdcard_adr : std_ulogic_vector(29 downto 0);

    begin
        litesdcard : litesdcard_core
            port map (
                clk           => system_clk,
                rst           => soc_rst,
                wb_ctrl_adr   => wb_sdcard_adr,
                wb_ctrl_dat_w => wb_ext_io_in.dat,
                wb_ctrl_dat_r => wb_sdcard_out.dat,
                wb_ctrl_sel   => wb_ext_io_in.sel,
                wb_ctrl_cyc   => wb_sdcard_cyc,
                wb_ctrl_stb   => wb_ext_io_in.stb,
                wb_ctrl_ack   => wb_sdcard_out.ack,
                wb_ctrl_we    => wb_ext_io_in.we,
                wb_ctrl_cti   => "000",
                wb_ctrl_bte   => "00",
                wb_ctrl_err   => open,
                wb_dma_adr    => wb_sddma_nr.adr,
                wb_dma_dat_w  => wb_sddma_nr.dat,
                wb_dma_dat_r  => wb_sddma_ir.dat,
                wb_dma_sel    => wb_sddma_nr.sel,
                wb_dma_cyc    => wb_sddma_nr.cyc,
                wb_dma_stb    => wb_sddma_nr.stb,
                wb_dma_ack    => wb_sddma_ir.ack,
                wb_dma_we     => wb_sddma_nr.we,
                wb_dma_cti    => open,
                wb_dma_bte    => open,
                wb_dma_err    => '0',
                sdcard_data   => sdcard_data(3 downto 0),
                sdcard_cmd    => sdcard_cmd,
                sdcard_clk    => sdcard_clk,
                -- card present is 0
                sdcard_cd     => '0',
                irq           => ext_irq_sdcard
                );

        -- Gate cyc with chip select from SoC
        wb_sdcard_cyc <= wb_ext_io_in.cyc and wb_ext_is_sdcard;

        wb_sdcard_adr <= x"0000" & wb_ext_io_in.adr(13 downto 0);

        wb_sdcard_out.stall <= not wb_sdcard_out.ack;

        -- Convert non-pipelined DMA wishbone to pipelined by suppressing
        -- non-acknowledged strobes
        process(system_clk)
        begin
            if rising_edge(system_clk) then
                wb_sddma_out <= wb_sddma_nr;
                if wb_sddma_stb_sent = '1' or
                    (wb_sddma_out.stb = '1' and wb_sddma_in.stall = '0') then
                    wb_sddma_out.stb <= '0';
                end if;
                if wb_sddma_nr.cyc = '0' or wb_sddma_ir.ack = '1' then
                    wb_sddma_stb_sent <= '0';
                elsif wb_sddma_in.stall = '0' then
                    wb_sddma_stb_sent <= wb_sddma_nr.stb;
                end if;
                wb_sddma_ir <= wb_sddma_in;
            end if;
        end process;

    end generate;

    -- Mux WB response on the IO bus
    wb_ext_io_out <= wb_eth_out when wb_ext_is_eth = '1' else
                     wb_sdcard_out when wb_ext_is_sdcard = '1' else
                     wb_dram_ctrl_out;

    do_rst_n <= ext_rst_n and not sw_rst;
    ext_rst_n <= '1';

    usr_led(0) <= gpio_out(0) when gpio_dir(0) = '1' else 'Z';
    usr_led(1) <= gpio_out(1) when gpio_dir(1) = '1' else 'Z';
    usr_led(2) <= gpio_out(2) when gpiob_dir(2) = '1' else 'Z';

    sys_pwrbtn_n <= gpio_out(8) when gpio_dir(8) = '1' else 'Z';
    gpio_in(8) <= sys_pwrbtn_n;

    sys_pwrok <= gpio_out(9) when gpio_dir(9) = '1' else 'Z';
    gpio_in(9) <= sys_pwrok;

    hpm_stby_en <= gpio_out(10) when gpio_dir(10) = '1' else 'Z';
    gpio_in(10) <= hpm_stby_en;

    -- spare pins J4 TPM header. Used for rework.
    -- pin 1
    spi0_cs_n <= gpio_out(16) when gpio_dir(16) = '1' else 'Z';
    gpio_in(16) <= spi0_cs_n;
    -- 2
    spi0_mosi <= gpio_out(17) when gpio_dir(17) = '1' else 'Z';
    gpio_in(17) <= spi0_mosi;
    -- 3 - used for litescope
    --spi0_miso <= gpio_out(18) when gpio_dir(18) = '1' else 'Z';
    --gpio_in(18) <= spi0_miso;
    -- 4 - used for litescope
    --spi0_clk <= gpio_out(19) when gpio_dir(19) = '1' else 'Z';
    --gpio_in(19) <= spi0_clk;
    -- 7
    tpm_pirq <= gpio_out(20) when gpio_dir(20) = '1' else 'Z';
    gpio_in(20) <= tpm_pirq;
    -- 8
    tpm_rst <= gpio_out(21) when gpio_dir(21) = '1' else 'Z';
    gpio_in(21) <= tpm_rst;
    -- 9
    tpm_gpio <= gpio_out(22) when gpio_dir(22) = '1' else 'Z';
    gpio_in(22) <= tpm_gpio;
    -- 10
    tpm_pp <= gpio_out(23) when gpio_dir(23) = '1' else 'Z';
    gpio_in(23) <= tpm_pp;

    gpio_in(30) <= '0';
    gpio_in(31) <= '1';

    -- FSI
    fsi_clk <= gpio_out(24) when gpio_dir(24) = '1' else 'Z';
    gpio_in(24) <= fsi_clk;
    fsi_dat <= gpio_out(25) when gpio_dir(25) = '1' else 'Z';
    gpio_in(25) <= fsi_dat;
    bmc_fsi_in_ena <= gpio_out(26) when gpio_dir(26) = '1' else 'Z';
    gpio_in(26) <= bmc_fsi_in_ena;

    ---- I2C lines
    --i2c_gpios: for i in 0 to 12 generate
    --begin
    --    -- i2c0 and i2c5 are not available through the interposer
    --    i2c_valid_gpios: if not (i = 0 or i = 5) generate
    --    begin
    --        i2c_scl(i) <= gpiob_out(2*i) when gpiob_dir(2*i) = '1' else 'Z';
    --        gpiob_in(2*i) <= i2c_scl(i);
    --        i2c_sda(i) <= gpiob_out(2*i+1) when gpiob_dir(2*i+1) = '1' else 'Z';
    --        gpiob_in(2*i+1) <= i2c_sda(i);
    --    end generate;
    --end generate;

    i2c_scl(3) <= gpiob_out(6) when gpiob_dir(6) = '1' else 'Z';
    gpiob_in(6) <= i2c_scl(3);
    i2c_sda(3) <= gpiob_out(7) when gpiob_dir(7) = '1' else 'Z';
    gpiob_in(7) <= i2c_sda(3);

    i2c_scl(4) <= gpiob_out(8) when gpiob_dir(8) = '1' else 'Z';
    gpiob_in(8) <= i2c_scl(4);
    i2c_sda(4) <= gpiob_out(9) when gpiob_dir(9) = '1' else 'Z';
    gpiob_in(9) <= i2c_sda(4);

    -- scl5 was wired incorrectly, use rework gpio instead
    --i2c_scl(5) <= gpiob_out(10) when gpiob_dir(10) = '1' else 'Z';
    --gpiob_in(10) <= i2c_scl(5);

    -- sda5 edge connector A12 goes to pcie_bmc_clk_100_p
    -- pcie_bmc_clk_100_p <= gpiob_out(11) when gpiob_dir(11) = '1' else 'Z';
    -- gpiob_in(11) <= pcie_bmc_clk_100_p;

    i2c_scl(6) <= gpiob_out(12) when gpiob_dir(12) = '1' else 'Z';
    gpiob_in(12) <= i2c_scl(6);
    i2c_sda(6) <= gpiob_out(13) when gpiob_dir(13) = '1' else 'Z';
    gpiob_in(13) <= i2c_sda(6);

    -- sda and scl are swapped by the interposer, we swap back here
    i2c_sda(10) <= gpiob_out(20) when gpiob_dir(20) = '1' else 'Z';
    gpiob_in(20) <= i2c_sda(10);
    i2c_scl(10) <= gpiob_out(21) when gpiob_dir(21) = '1' else 'Z';
    gpiob_in(21) <= i2c_scl(10);

    -- sda and scl are swapped by the interposer, we swap back here
    i2c_sda(11) <= gpiob_out(22) when gpiob_dir(22) = '1' else 'Z';
    gpiob_in(22) <= i2c_sda(11);
    i2c_scl(11) <= gpiob_out(23) when gpiob_dir(23) = '1' else 'Z';
    gpiob_in(23) <= i2c_scl(11);

    i2c_scl(12) <= gpiob_out(24) when gpiob_dir(24) = '1' else 'Z';
    gpiob_in(24) <= i2c_scl(12);
    i2c_sda(12) <= gpiob_out(25) when gpiob_dir(25) = '1' else 'Z';
    gpiob_in(25) <= i2c_sda(12);

    -- host i2c13 uses dcscm i2c2
    i2c_scl(2) <= gpiob_out(26) when gpiob_dir(26) = '1' else 'Z';
    gpiob_in(26) <= i2c_scl(2);
    i2c_sda(2) <= gpiob_out(27) when gpiob_dir(27) = '1' else 'Z';
    gpiob_in(27) <= i2c_sda(2);

    -- host i2c14 uses dcscm i2c1
    i2c_scl(1) <= gpiob_out(28) when gpiob_dir(28) = '1' else 'Z';
    gpiob_in(28) <= i2c_scl(1);
    i2c_sda(1) <= gpiob_out(29) when gpiob_dir(29) = '1' else 'Z';
    gpiob_in(29) <= i2c_sda(1);

    i2c_scl <= (others => 'Z');
    i2c_sda <= (others => 'Z');

end architecture behaviour;
