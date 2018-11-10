// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#![feature(async_await, await_macro, futures_api, pin, arbitrary_self_types)]
#![deny(warnings)]

use {
    byteorder::{LittleEndian, ReadBytesExt},
    fidl_fuchsia_wlan_device as wlan_device,
    fidl_fuchsia_wlan_mlme as wlan_mlme,
    fidl_fuchsia_wlan_tap as wlantap,
    fuchsia_async as fasync,
    fuchsia_zircon::prelude::*,
    futures::{prelude::*},
    std::io::Cursor,
    std::sync::{Arc, Mutex},
    wlantap_client::Wlantap,
};

mod mac_frames;

#[cfg(test)]
mod eth_frames;
#[cfg(test)]
mod test_utils;

const HW_MAC_ADDR: [u8; 6] = [0x67, 0x62, 0x6f, 0x6e, 0x69, 0x6b];
const BSSID: [u8; 6] = [0x62, 0x73, 0x73, 0x62, 0x73, 0x73];

fn create_2_4_ghz_band_info() -> wlan_device::BandInfo {
    wlan_device::BandInfo{
        band_id: wlan_mlme::Band::WlanBand2Ghz,
        ht_caps: Some(Box::new(wlan_mlme::HtCapabilities {
            ht_cap_info: wlan_mlme::HtCapabilityInfo {
                ldpc_coding_cap: false,
                chan_width_set: wlan_mlme::ChanWidthSet::TwentyForty as u8,
                sm_power_save: wlan_mlme::SmPowerSave::Disabled as u8,
                greenfield: true,
                short_gi_20: true,
                short_gi_40: true,
                tx_stbc: true,
                rx_stbc: 1,
                delayed_block_ack: false,
                max_amsdu_len: wlan_mlme::MaxAmsduLen::Octets3839 as u8,
                dsss_in_40: false,
                intolerant_40: false,
                lsig_txop_protect: false,
            },
            ampdu_params: wlan_mlme::AmpduParams {
                exponent: 0,
                min_start_spacing: wlan_mlme::MinMpduStartSpacing::NoRestrict as u8,
            },
            mcs_set: wlan_mlme::SupportedMcsSet {
                rx_mcs_set: 0x01000000ff,
                rx_highest_rate: 0,
                tx_mcs_set_defined: true,
                tx_rx_diff: false,
                tx_max_ss: 1,
                tx_ueqm: false,
            },
            ht_ext_cap: wlan_mlme::HtExtCapabilities {
                pco: false,
                pco_transition: wlan_mlme::PcoTransitionTime::PcoReserved as u8,
                mcs_feedback: wlan_mlme::McsFeedback::McsNofeedback as u8,
                htc_ht_support: false,
                rd_responder: false,
            },
            txbf_cap: wlan_mlme::TxBfCapability {
                implicit_rx: false,
                rx_stag_sounding: false,
                tx_stag_sounding: false,
                rx_ndp: false,
                tx_ndp: false,
                implicit: false,
                calibration: wlan_mlme::Calibration::CalibrationNone as u8,
                csi: false,
                noncomp_steering: false,
                comp_steering: false,
                csi_feedback: wlan_mlme::Feedback::FeedbackNone as u8,
                noncomp_feedback: wlan_mlme::Feedback::FeedbackNone as u8,
                comp_feedback: wlan_mlme::Feedback::FeedbackNone as u8,
                min_grouping: wlan_mlme::MinGroup::MinGroupOne as u8,
                csi_antennas: 1,
                noncomp_steering_ants: 1,
                comp_steering_ants: 1,
                csi_rows: 1,
                chan_estimation: 1,
            },
            asel_cap: wlan_mlme::AselCapability {
                asel: false,
                csi_feedback_tx_asel: false,
                ant_idx_feedback_tx_asel: false,
                explicit_csi_feedback: false,
                antenna_idx_feedback: false,
                rx_asel: false,
                tx_sounding_ppdu: false,
            }

        })),
        vht_caps: None,
        basic_rates: vec![2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108],
        supported_channels: wlan_device::ChannelList{
            base_freq: 2407,
            channels: vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]
        }
    }
}

fn create_wlantap_config() -> wlantap::WlantapPhyConfig {
    use fidl_fuchsia_wlan_device::{DriverFeature, SupportedPhy};
    wlantap::WlantapPhyConfig {
        phy_info: wlan_device::PhyInfo{
            id: 0,
            dev_path: None,
            hw_mac_address: HW_MAC_ADDR,
            supported_phys: vec![
                SupportedPhy::Dsss, SupportedPhy::Cck, SupportedPhy::Ofdm, SupportedPhy::Ht
            ],
            driver_features: vec![DriverFeature::Synth, DriverFeature::TxStatusReport],
            mac_roles: vec![wlan_device::MacRole::Client],
            caps: vec![],
            bands: vec![
                create_2_4_ghz_band_info()
            ]
        },
        name: String::from("wlantap0"),
        quiet: false,
    }
}

struct State {
    current_channel: wlan_mlme::WlanChan,
    frame_buf: Vec<u8>,
    is_associated: bool,
}

impl State {
    fn new() -> Self {
        Self {
            current_channel: wlan_mlme::WlanChan {
                primary: 0,
                cbw: wlan_mlme::Cbw::Cbw20,
                secondary80: 0
            },
            frame_buf: vec![],
            is_associated: false,
        }
    }
}

fn send_beacon(frame_buf: &mut Vec<u8>, channel: &wlan_mlme::WlanChan, bss_id: &[u8; 6],
               ssid: &[u8], proxy: &wlantap::WlantapPhyProxy)
    -> Result<(), failure::Error>
{
    frame_buf.clear();
    mac_frames::MacFrameWriter::<&mut Vec<u8>>::new(frame_buf)
        .beacon(
            &mac_frames::MgmtHeader{
                frame_control: mac_frames::FrameControl(0), // will be filled automatically
                duration: 0,
                addr1: mac_frames::BROADCAST_ADDR.clone(),
                addr2: bss_id.clone(),
                addr3: bss_id.clone(),
                seq_control: mac_frames::SeqControl {
                    frag_num: 0,
                    seq_num: 123
                },
                ht_control: None
            },
            &mac_frames::BeaconFields{
                timestamp: 0,
                beacon_interval: 100,
                capability_info: mac_frames::CapabilityInfo(0),
            })?
        .ssid(ssid)?
        .supported_rates(&[0x82, 0x84, 0x8b, 0x0c, 0x12, 0x96, 0x18, 0x24])?
        .dsss_parameter_set(channel.primary)?;

    let rx_info = &mut create_rx_info(channel);
    proxy.rx(0, &mut frame_buf.iter().cloned(), rx_info)?;
    Ok(())
}

fn send_authentication(frame_buf: &mut Vec<u8>, channel: &wlan_mlme::WlanChan, bss_id: &[u8; 6],
                       proxy: &wlantap::WlantapPhyProxy)
    -> Result<(), failure::Error>
{
    frame_buf.clear();
    mac_frames::MacFrameWriter::<&mut Vec<u8>>::new(frame_buf).authentication(
        &mac_frames::MgmtHeader {
            frame_control: mac_frames::FrameControl(0), // will be filled automatically
            duration: 0,
            addr1: HW_MAC_ADDR,
            addr2: bss_id.clone(),
            addr3: bss_id.clone(),
            seq_control: mac_frames::SeqControl {
                frag_num: 0,
                seq_num: 123,
            },
            ht_control: None,
        },
        &mac_frames::AuthenticationFields {
            auth_algorithm_number: mac_frames::AuthAlgorithm::OpenSystem as u16,
            auth_txn_seq_number: 2, // Always 2 for successful authentication
            status_code: mac_frames::StatusCode::Success as u16,
        },
    )?;

    let rx_info = &mut create_rx_info(channel);
    proxy.rx(0, &mut frame_buf.iter().cloned(), rx_info)?;
    Ok(())
}

fn send_association_response(
    frame_buf: &mut Vec<u8>, channel: &wlan_mlme::WlanChan, bss_id: &[u8; 6],
    proxy: &wlantap::WlantapPhyProxy,
) -> Result<(), failure::Error> {
    frame_buf.clear();
    let mut cap_info = mac_frames::CapabilityInfo(0);
    cap_info.set_ess(true);
    cap_info.set_short_preamble(true);
    mac_frames::MacFrameWriter::<&mut Vec<u8>>::new(frame_buf).association_response(
        &mac_frames::MgmtHeader {
            frame_control: mac_frames::FrameControl(0), // will be filled automatically
            duration: 0,
            addr1: HW_MAC_ADDR,
            addr2: bss_id.clone(),
            addr3: bss_id.clone(),
            seq_control: mac_frames::SeqControl {
                frag_num: 0,
                seq_num: 123,
            },
            ht_control: None,
        },
        &mac_frames::AssociationResponseFields {
            capability_info: cap_info,
            status_code: 0,    // Success
            association_id: 2, // Can be any
        },
    )?
    .supported_rates(&[0x82, 0x84, 0x8b, 0x0c, 0x12, 0x96, 0x18, 0x24])?
    .extended_supported_rates(&[48, 72, 96, 108])?;

    let rx_info = &mut create_rx_info(channel);
    proxy.rx(0, &mut frame_buf.iter().cloned(), rx_info)?;
    Ok(())
}

fn create_rx_info(channel: &wlan_mlme::WlanChan) -> wlantap::WlanRxInfo {
    wlantap::WlanRxInfo {
        rx_flags: 0,
        valid_fields: 0,
        phy: 0,
        data_rate: 0,
        chan: wlan_mlme::WlanChan { // TODO(FIDL-54): use clone()
            primary: channel.primary,
            cbw: channel.cbw,
            secondary80: channel.secondary80
        },
        mcs: 0,
        rssi_dbm: 0,
        rcpi_dbmh: 0,
        snr_dbh: 0,
    }
}

fn get_frame_ctrl(packet_data: Vec<u8>) -> mac_frames::FrameControl {
    let mut reader = Cursor::new(packet_data);
    let frame_ctrl = reader.read_u16::<LittleEndian>().unwrap();
    mac_frames::FrameControl(frame_ctrl)
}

fn handle_tx(args: wlantap::TxArgs, state: &mut State, proxy: &wlantap::WlantapPhyProxy) {
    let frame_ctrl = get_frame_ctrl(args.packet.data);
    if frame_ctrl.typ() == mac_frames::FrameControlType::Mgmt as u16 {
        handle_mgmt_tx(frame_ctrl.subtype(), state, proxy);
    }
}

fn handle_mgmt_tx(subtype: u16, state: &mut State, proxy: &wlantap::WlantapPhyProxy) {
    if subtype == mac_frames::MgmtSubtype::Authentication as u16 {
        println!("Authentication received.");
        send_authentication(&mut state.frame_buf, &state.current_channel, &BSSID, proxy)
            .expect("Error sending fake authentication frame.");
        println!("Authentication sent.");
    } else if subtype == mac_frames::MgmtSubtype::AssociationRequest as u16 {
        println!("Association Request received.");
        send_association_response(&mut state.frame_buf, &state.current_channel, &BSSID, proxy)
            .expect("Error sending fake association response frame.");
        println!("Association Response sent.");
        state.is_associated = true;
    }
}

fn main() -> Result<(), failure::Error> {
    let mut exec = fasync::Executor::new().expect("error creating executor");
    let wlantap = Wlantap::open().expect("error with Wlantap::open()");
    let state = Arc::new(Mutex::new(State::new()));
    let proxy = wlantap.create_phy(create_wlantap_config()).expect("error creating wlantap config");

    let event_listener = event_listener(state.clone(), proxy.clone());
    let beacon_timer = beacon_sender(state.clone(), proxy.clone());
    println!("Hardware simlulator started. Try to scan or connect to \"fakenet\"");

    exec.run_singlethreaded(event_listener.join(beacon_timer));
    Ok(())
}

async fn event_listener(state: Arc<Mutex<State>>, proxy: wlantap::WlantapPhyProxy) {
    let mut events = proxy.take_event_stream();
    while let Some(event) = await!(events.try_next()).unwrap() {
        match event {
            wlantap::WlantapPhyEvent::SetChannel { args } => {
                let mut state = state.lock().unwrap();
                state.current_channel = args.chan;
                println!("setting channel to {:?}", state.current_channel);
            }
            wlantap::WlantapPhyEvent::Tx { args } => {
                let mut state = state.lock().unwrap();
                handle_tx(args, &mut state, &proxy);
            }
            _ => {}
        }
    }
}

async fn beacon_sender(state: Arc<Mutex<State>>, proxy: wlantap::WlantapPhyProxy) {
    let mut beacon_timer_stream = fasync::Interval::new(102_400_000.nanos());
    while let Some(_) = await!(beacon_timer_stream.next()) {
        let state = &mut *state.lock().unwrap();
        if state.current_channel.primary == 6 {
            if !state.is_associated { eprintln!("sending beacon!"); }
            send_beacon(&mut state.frame_buf, &state.current_channel, &BSSID,
                        "fakenet".as_bytes(), &proxy).unwrap();
        }
    }
}

#[cfg(test)]
mod simulation_tests {
    use super::*;
    use {
        fidl_fuchsia_wlan_service as fidl_wlan_service,
        failure::format_err,
        fuchsia_app as app,
        fuchsia_zircon as zx,
        futures::{channel::mpsc, poll, select},
        pin_utils::pin_mut,
        std::{collections::HashMap, fs::{self, File}, panic},
    };

    const BSS_FOO: [u8; 6] = [0x62, 0x73, 0x73, 0x66, 0x6f, 0x6f];
    const SSID_FOO: &[u8] = b"foo";
    const BSS_BAR: [u8; 6] = [0x62, 0x73, 0x73, 0x62, 0x61, 0x72];
    const SSID_BAR: &[u8] = b"bar";
    const BSS_BAZ: [u8; 6] = [0x62, 0x73, 0x73, 0x62, 0x61, 0x7a];
    const SSID_BAZ: &[u8] = b"baz";

    // TODO(NET-1885) - removed #[allow(dead_code)] once `connecting_to_ap` test is fixed
    #[allow(dead_code)]
    const CHANNEL: wlan_mlme::WlanChan = wlan_mlme::WlanChan {
        primary: 6,
        secondary80: 0,
        cbw: wlan_mlme::Cbw::Cbw20,
    };

    // Temporary workaround to run tests synchronously. This is because wlan service only works with
    // one PHY, so having tests with multiple PHYs running in parallel make them flaky.
    #[test]
    fn ethernet_and_scan_and_minstrel() {
        let mut ok = true;
        ok = run_test("verify_ethernet", verify_ethernet) && ok;
        ok = run_test("simulate_scan", simulate_scan) && ok;
        // TODO(NET-1885) - commenting out due to flake
        // ok = run_test("connecting_to_ap", connecting_to_ap) && ok;
        ok = run_test("verify_rate_selection", verify_rate_selection) && ok;
        assert!(ok);
    }

    // test
    fn verify_ethernet() {
        let mut exec = fasync::Executor::new().expect("Failed to create an executor");
        // Make sure there is no existing ethernet device.
        let client = exec.run_singlethreaded(create_eth_client(&HW_MAC_ADDR))
            .expect(&format!("creating ethernet client: {:?}", &HW_MAC_ADDR));
        assert!(client.is_none());
        // Create wlan_tap device which will in turn create ethernet device.
        let mut helper = test_utils::TestHelper::begin_test(&mut exec, create_wlantap_config());
        let mut retry = test_utils::RetryWithBackoff::new(5.seconds());
        loop {
            let client = exec.run_singlethreaded(create_eth_client(&HW_MAC_ADDR))
                .expect(&format!("creating ethernet client: {:?}", &HW_MAC_ADDR));
            if client.is_some() { break; }
            let slept = retry.sleep_unless_timed_out();
            assert!(slept, "No ethernet client with mac_addr {:?} found in time", &HW_MAC_ADDR);
        }
        let wlan_service = app::client::connect_to_service::<fidl_wlan_service::WlanMarker>()
            .expect("connecting to wlan service");
        loop_until_iface_is_found(&mut exec, &wlan_service, &mut helper);
    }

    fn clear_ssid_and_ensure_iface_gone() {
        let mut exec = fasync::Executor::new().expect("Failed to create an executor");
        let wlan_service = app::client::connect_to_service::<fidl_wlan_service::WlanMarker>()
            .expect("Failed to connect to wlan service");
        exec.run_singlethreaded(wlan_service.clear_saved_networks()).expect("Clearing SSID");

        let mut retry = test_utils::RetryWithBackoff::new(5.seconds());
        loop {
            let status = exec.run_singlethreaded(wlan_service.status())
                .expect("error getting status() from wlan_service");
            if status.error.code == fidl_wlan_service::ErrCode::NotFound { return; }
            let slept = retry.sleep_unless_timed_out();
            assert!(slept, "The interface was not removed in time");
        }
    }

    // test
    fn simulate_scan() {
        let mut exec = fasync::Executor::new().expect("Failed to create an executor");
        let mut helper =
            test_utils::TestHelper::begin_test(&mut exec, create_wlantap_config());

        let wlan_service = app::client::connect_to_service::<fidl_wlan_service::WlanMarker>()
            .expect("Failed to connect to wlan service");

        let proxy = helper.proxy();
        let scan_result = scan(&mut exec, &wlan_service, &proxy, &mut helper);

        assert_eq!(fidl_wlan_service::ErrCode::Ok, scan_result.error.code,
                   "The error message was: {}", scan_result.error.description);
        let mut aps:Vec<_> = scan_result.aps.expect("Got empty scan results")
            .into_iter().map(|ap| (ap.ssid, ap.bssid)).collect();
        aps.sort();
        let mut expected_aps = [
            ( String::from_utf8_lossy(SSID_FOO).to_string(), BSS_FOO.to_vec() ),
            ( String::from_utf8_lossy(SSID_BAR).to_string(), BSS_BAR.to_vec() ),
            ( String::from_utf8_lossy(SSID_BAZ).to_string(), BSS_BAZ.to_vec() ),
        ];
        expected_aps.sort();
        assert_eq!(&expected_aps, &aps[..]);
    }

    // TODO(NET-1885) - removed #[allow(dead_code)] once test is fixed
    #[allow(dead_code)]
    // test
    fn connecting_to_ap() {
        let mut exec = fasync::Executor::new().expect("Failed to create an executor");
        let mut helper = test_utils::TestHelper::begin_test(&mut exec, create_wlantap_config());

        let wlan_service = app::client::connect_to_service::<fidl_wlan_service::WlanMarker>()
            .expect("Failed to connect to wlan service");
        let proxy = helper.proxy();
        loop_until_iface_is_found(&mut exec, &wlan_service, &mut helper);

        connect(&mut exec, &wlan_service, &proxy, &mut helper, SSID_FOO, &BSS_FOO);

        let status = status(&mut exec, &wlan_service, &mut helper);
        assert_eq!(status.error.code, fidl_wlan_service::ErrCode::Ok);
        assert_eq!(status.state, fidl_wlan_service::State::Associated);
        let ap = status.current_ap.expect("expect to be associated to an AP");
        assert_eq!(ap.bssid, BSS_FOO.to_vec());
        assert_eq!(ap.ssid, String::from_utf8_lossy(SSID_FOO).to_string());
        assert_eq!(ap.chan, CHANNEL);
        assert!(ap.is_compatible);
        assert!(!ap.is_secure);
    }

    fn loop_until_iface_is_found(exec: &mut fasync::Executor,
                                 wlan_service: &fidl_wlan_service::WlanProxy,
                                 helper: &mut test_utils::TestHelper) {
        let mut retry = test_utils::RetryWithBackoff::new(5.seconds());
        loop {
            let status = status(exec, wlan_service, helper);
            if status.error.code != fidl_wlan_service::ErrCode::Ok {
                let slept = retry.sleep_unless_timed_out();
                assert!(slept, "Wlanstack did not recognize the interface in time");
            } else {
                return
            }
        }
    }

    fn status(exec: &mut fasync::Executor, wlan_service: &fidl_wlan_service::WlanProxy,
              helper: &mut test_utils::TestHelper)
              -> fidl_wlan_service::WlanStatus {
        helper.run(exec, 1.seconds(), "status request", |_| {}, wlan_service.status())
            .expect("expect wlan status")
    }

    fn scan(exec: &mut fasync::Executor,
            wlan_service: &fidl_wlan_service::WlanProxy,
            phy: &wlantap::WlantapPhyProxy,
            helper: &mut test_utils::TestHelper) -> fidl_wlan_service::ScanResult {
        let mut wlanstack_retry = test_utils::RetryWithBackoff::new(5.seconds());
        loop {
            let scan_result = helper.run(exec, 10.seconds(), "receive a scan response",
               |event| {
                   match event {
                       wlantap::WlantapPhyEvent::SetChannel { args } => {
                           println!("set channel to {:?}", args.chan);
                           if args.chan.primary == 1 {
                               send_beacon(&mut vec![], &args.chan, &BSS_FOO, SSID_FOO, &phy)
                                   .unwrap();
                           } else if args.chan.primary == 6 {
                               send_beacon(&mut vec![], &args.chan, &BSS_BAR, SSID_BAR, &phy)
                                   .unwrap();
                           } else if args.chan.primary == 11 {
                               send_beacon(&mut vec![], &args.chan, &BSS_BAZ, SSID_BAZ, &phy)
                                   .unwrap();
                           }
                       },
                       _ => {}
                   }
               },
               wlan_service.scan(&mut fidl_wlan_service::ScanRequest { timeout: 5 })).unwrap();
            if scan_result.error.code == fidl_wlan_service::ErrCode::NotFound {
                let slept = wlanstack_retry.sleep_unless_timed_out();
                assert!(slept, "Wlanstack did not recognize the interface in time");
            } else {
                return scan_result;
            }
        }
    }

    fn create_connect_config(ssid: &[u8], bssid: &[u8; 6]) -> fidl_wlan_service::ConnectConfig {
        fidl_wlan_service::ConnectConfig {
            ssid: String::from_utf8_lossy(ssid).to_string(),
            pass_phrase: "".to_string(),
            scan_interval: 5,
            bssid: String::from_utf8_lossy(bssid).to_string(),
        }
    }

    fn connect(exec: &mut fasync::Executor,
               wlan_service: &fidl_wlan_service::WlanProxy,
               phy: &wlantap::WlantapPhyProxy,
               helper: &mut test_utils::TestHelper, ssid: &[u8], bssid: &[u8;6]) {
        let mut connect_config = create_connect_config(ssid, bssid);
        let connect_fut = wlan_service.connect(&mut connect_config);
        let error = helper.run(exec, 10.seconds(),
                           &format!("connect to {}({:2x?})", String::from_utf8_lossy(ssid), bssid),
                           |event| { handle_connect_events(event, &phy, ssid, bssid); },
                           connect_fut).unwrap();
        assert_eq!(error.code, fidl_wlan_service::ErrCode::Ok, "connect failed: {:?}", error);
    }

    fn handle_connect_events(event: wlantap::WlantapPhyEvent, phy: &wlantap::WlantapPhyProxy,
                             ssid: &[u8], bssid: &[u8; 6]) {
        match event {
            wlantap::WlantapPhyEvent::SetChannel { args } => {
                println!("channel: {:?}", args.chan);
                if args.chan.primary == CHANNEL.primary {
                    send_beacon(&mut vec![], &args.chan, bssid, ssid, &phy).unwrap();
                }
            }
            wlantap::WlantapPhyEvent::Tx { args } => {
                let frame_ctrl = get_frame_ctrl(args.packet.data);
                if frame_ctrl.typ() == mac_frames::FrameControlType::Mgmt as u16 {
                    let subtyp = frame_ctrl.subtype();
                    if subtyp == mac_frames::MgmtSubtype::Authentication as u16 {
                        send_authentication(&mut vec![], &CHANNEL, bssid, &phy)
                            .expect("Error sending fake authentication frame.");
                    } else if subtyp == mac_frames::MgmtSubtype::AssociationRequest as u16 {
                        send_association_response(&mut vec![], &CHANNEL, bssid, &phy)
                            .expect("Error sending fake association response frame.");
                    }
                }
            },
            _ => {},
        }
    }

    const BSS_MINSTL : [u8; 6] = [0x6d, 0x69, 0x6e, 0x73, 0x74, 0x0a];
    const SSID_MINSTREL : &[u8] = b"minstrel";
    // test
    fn verify_rate_selection() {
        let mut exec = fasync::Executor::new().expect("error creating executor");
        let wlan_service = app::client::connect_to_service::<fidl_wlan_service::WlanMarker>()
            .expect("Error connecting to wlan service");
        let mut helper =
            test_utils::TestHelper::begin_test(&mut exec, wlantap::WlantapPhyConfig{
                quiet: true,
                ..create_wlantap_config()
            });

        loop_until_iface_is_found(&mut exec, &wlan_service, &mut helper);

        let phy = helper.proxy();
        connect(&mut exec, &wlan_service, &phy, &mut helper, SSID_MINSTREL, &BSS_MINSTL);

        let beacon_sender_fut = beacon_sender(&BSS_MINSTL, SSID_MINSTREL, &phy).fuse();
        pin_mut!(beacon_sender_fut);

        let (sender, mut receiver) = mpsc::channel(1);
        let eth_sender_fut = eth_sender(&mut receiver).fuse();
        pin_mut!(eth_sender_fut);

        let test_fut = async {
            select! {
                _ = eth_sender_fut => Ok(()),
                _ = beacon_sender_fut => Err(format_err!("beacon sender should not have exited")),
            }
        };
        pin_mut!(test_fut);

        let threshold = 130;
        let will_succeed = |idx| {129 <= idx && idx <= threshold};
        let is_done = |hm: &HashMap<u16, u64>| {
            let second_largest = hm.iter().map(|(k, v)| if k == &threshold {0} else {*v}).max()
                .unwrap_or(0);
            let expected = hm.get(&threshold).unwrap_or(&0);
            second_largest > 0 && *expected >= 25 * second_largest
        };
        let mut hm = HashMap::new();
        let () = helper.run(&mut exec, 30.seconds(), "verify rate selection is working",
                               |event| {
                                   handle_rate_selection_event(event, &phy, &BSS_MINSTL, &mut hm,
                                                               will_succeed, is_done,
                                                               sender.clone());
                               },
                               test_fut).expect("running main future");

        let total = hm.values().sum::<u64>();
        let others = total - hm[&threshold];
        println!("{:#?}\ntotal: {}, others: {}", hm, total, others);
        // Minstrel has tried all options
        assert_eq!(7, hm.keys().count());
        let max = hm.keys().max_by_key(|k| {hm[&k]}).unwrap();
        // The dominant one is indeed the optimal.
        assert_eq!(max, &threshold);
    }

    fn handle_rate_selection_event<F, G>(event: wlantap::WlantapPhyEvent,
                                         phy: &wlantap::WlantapPhyProxy, bssid: &[u8; 6],
                                         hm: &mut HashMap<u16, u64>, should_succeed: F, is_done: G,
                                         mut sender: mpsc::Sender<()>)
    where F: Fn(u16) -> bool, G: Fn(&HashMap<u16, u64>) -> bool {
        match event {
            wlantap::WlantapPhyEvent::Tx { args } => {
                let frame_ctrl = get_frame_ctrl(args.packet.data);
                if frame_ctrl.typ() == mac_frames::FrameControlType::Data as u16 {
                    let tx_vec_idx = args.packet.info.tx_vector_idx;
                    send_tx_status_report(*bssid, tx_vec_idx, should_succeed(tx_vec_idx), phy)
                        .expect("Error sending tx_status report");
                    let count = hm.entry(tx_vec_idx).or_insert(0);
                    *count += 1;
                    if is_done(hm) { sender.try_send(()).expect("Indicating test successful"); }
                }
            },
            _ => {},
        }
    }

    async fn eth_sender(receiver: &mut mpsc::Receiver<()>) -> Result<(), failure::Error> {
        let mut client = await!(create_eth_client(&HW_MAC_ADDR))
            .expect("cannot create ethernet client")
            .expect(&format!("ethernet client not found {:?}", &HW_MAC_ADDR));

        let mut buf : Vec<u8> = vec![];
        eth_frames::write_eth_header(&mut buf, &eth_frames::EthHeader{
            dst : BSSID,
            src : HW_MAC_ADDR,
            eth_type : eth_frames::EtherType::Ipv4 as u16,
        }).expect("Error creating fake ethernet frame");

        const ETH_PACKETS_PER_INTERVAL : u64 = 8;

        let mut timer_stream = fasync::Interval::new(7.millis())
        .map(|()| stream::repeat(()).take(ETH_PACKETS_PER_INTERVAL))
        .flatten();
        let mut client_stream = client.get_stream().fuse();
        'eth_sender: loop {
            select! {
                event = client_stream.next() => {
                    if let Some(Ok(ethernet::Event::StatusChanged)) = event {
                        println!("status changed to: {:?}", await!(client.get_status()).unwrap());
                    }
                },
                () = timer_stream.select_next_some() => { client.send(&buf); },
            }
            if let Ok(Some(())) = receiver.try_next() { break 'eth_sender; }
        }
        // Send any packets that are still in the buffer
        poll!(client_stream.next());
        Ok(())
    }

    fn create_wlan_tx_status_entry(tx_vec_idx: u16) -> wlantap::WlanTxStatusEntry {
        fidl_fuchsia_wlan_tap::WlanTxStatusEntry{
            tx_vec_idx : tx_vec_idx,
            attempts: 1,
        }
    }

    fn send_tx_status_report(bssid: [u8; 6], tx_vec_idx: u16, is_successful: bool,
                             proxy: &wlantap::WlantapPhyProxy) -> Result<(), failure::Error> {
        use fidl_fuchsia_wlan_tap::WlanTxStatus;

        let mut ts = WlanTxStatus {
            peer_addr : bssid,
            success: is_successful,
            tx_status_entries: [
                create_wlan_tx_status_entry(tx_vec_idx),
                create_wlan_tx_status_entry(0),
                create_wlan_tx_status_entry(0),
                create_wlan_tx_status_entry(0),
                create_wlan_tx_status_entry(0),
                create_wlan_tx_status_entry(0),
                create_wlan_tx_status_entry(0),
                create_wlan_tx_status_entry(0),
            ],
        };
        proxy.report_tx_status(0, &mut ts)?;
        Ok(())
    }

    async fn create_eth_client(mac: &[u8; 6]) -> Result<Option<ethernet::Client>, failure::Error> {
        const ETH_PATH : &str = "/dev/class/ethernet";
        let files = fs::read_dir(ETH_PATH)?;
        for file in files {
            let vmo = zx::Vmo::create_with_opts(
                zx::VmoOptions::NON_RESIZABLE,
                256 * ethernet::DEFAULT_BUFFER_SIZE as u64)?;

            let path = file?.path();
            let dev = File::open(path)?;
            if let Ok(client) = await !(ethernet::Client::from_file(dev, vmo,
            ethernet::DEFAULT_BUFFER_SIZE, "wlan-hw-sim")) {
                if let Ok(info) = await !(client.info()) {
                    if &info.mac.octets == mac {
                        println!("ethernet client created: {:?}", client);
                        await!(client.start()).expect("error starting ethernet device");
                        // must call get_status() after start() to clear
                        // zx::Signals::USER_0 otherwise there will be a stream
                        // of infinite StatusChanged events that blocks
                        // fasync::Interval
                        println!("info: {:?} status: {:?}",
                                  await!(client.info()).expect("calling client.info()"),
                                  await!(client.get_status()).expect("getting client status()"));
                        return Ok(Some(client));
                      }
                  }
              }
        }
        Ok(None)
    }

    async fn beacon_sender<'a>(bssid: &'a[u8; 6], ssid: &'a[u8], phy: &'a wlantap::WlantapPhyProxy)
        ->Result<(), failure::Error> {
        let mut beacon_timer_stream = fasync::Interval::new (102_400_000.nanos());
        while let Some(_) = await !(beacon_timer_stream.next()) {
            send_beacon(&mut vec ![], &CHANNEL, bssid, ssid, &phy).unwrap();
        }
        Ok(())
    }

    fn run_test<F>(name: &str, f: F) -> bool
        where F: FnOnce() + panic::UnwindSafe
    {
        println!("\nTest `{}` started\n", name);
        let result = panic::catch_unwind(f);
        clear_ssid_and_ensure_iface_gone();
        match result {
            Ok(_) => {
                println!("Test `{}` passed\n", name);
                true
            },
            Err(_) => {
                println!("Test `{}` failed\n", name);
                false
            }
        }
    }
}
