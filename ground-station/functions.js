const electron = require('electron');
const fs = require('fs');
var ox_tank_mass = [];
var ox_temp1 = [];
var ox_temp2 = [];
var pre_inj_temp = [];
var post_inj_temp = [];
var ox_tank_pres = [];
var pre_inj_pres = [];
var comb_chamb_pres = [];
var flight_cont;
var igniter_cont;

window.onload = function () {

  // Mass target submission
  const mass = document.querySelector('mass');
  document.getElementById("btn_submit").addEventListener("click", function submitMass(e) {
    e.preventDefault();
    var masstarget = document.getElementById('mass_target').value;
    console.log(masstarget);
    document.getElementById("mass_target").value = "";
  });

  // Changing sensors/transducers
  document.getElementById("db1_val1").addEventListener('click', function toggle11(e) {
    e.preventDefault();
    document.getElementById("dv1").innerHTML = "Ox. Temp 1";
    document.getElementById("li1").value = "oxtemp1";
  });

  document.getElementById("db1_val2").addEventListener('click', function toggle12(e) {
    e.preventDefault();
    document.getElementById("dv1").innerHTML = "Ox. Temp 2";
    document.getElementById("li1").value = "oxtemp2";
  });

  document.getElementById("db1_val3").addEventListener('click', function toggle13(e) {
    e.preventDefault();
    document.getElementById("dv1").innerHTML = "Pre-Injector Temp";
    document.getElementById("li1").value = "preinjtemp";
  });

  document.getElementById("db1_val4").addEventListener('click', function toggle14(e) {
    e.preventDefault();
    document.getElementById("dv1").innerHTML = "Post-Injector Temp";
    document.getElementById("li1").value = "postinjtemp";
  });

  document.getElementById("db2_val1").addEventListener('click', function toggle21(e) {
    e.preventDefault();
    document.getElementById("dv2").innerHTML = "Ox. Temp 1";
    document.getElementById("li2").value = "oxtemp1";
  });

  document.getElementById("db2_val2").addEventListener('click', function toggle22(e) {
    e.preventDefault();
    document.getElementById("dv2").innerHTML = "Ox. Temp 2";
    document.getElementById("li2").value = "oxtemp2";
  });

  document.getElementById("db2_val3").addEventListener('click', function toggle23(e) {
    e.preventDefault();
    document.getElementById("dv2").innerHTML = "Pre-Injector Temp";
    document.getElementById("li2").value = "preinjtemp";
  });

  document.getElementById("db2_val4").addEventListener('click', function toggle24(e) {
    e.preventDefault();
    document.getElementById("dv2").innerHTML = "Post-Injector Temp";
    document.getElementById("li2").value = "postinjtemp";
  });

  document.getElementById("db3_val1").addEventListener('click', function toggle31(e) {
    e.preventDefault();
    document.getElementById("dv3").innerHTML = "Ox. Temp 1";
    document.getElementById("li3").value = "oxtemp3";
  });

  document.getElementById("db3_val2").addEventListener('click', function toggle32(e) {
    e.preventDefault();
    document.getElementById("dv3").innerHTML = "Ox. Temp 2";
    document.getElementById("li3").value = "oxtemp2";
  });

  document.getElementById("db3_val3").addEventListener('click', function toggle33(e) {
    e.preventDefault();
    document.getElementById("dv3").innerHTML = "Pre-Injector Temp";
    document.getElementById("li3").value = "preinjtemp";
  });

  document.getElementById("db3_val4").addEventListener('click', function toggle34(e) {
    e.preventDefault();
    document.getElementById("dv3").innerHTML = "Post-Injector Temp";
    document.getElementById("li3").value = "postinjtemp";
  });

  document.getElementById("db4_val1").addEventListener('click', function toggle41(e) {
    e.preventDefault();
    document.getElementById("dv4").innerHTML = "Ox. Temp 1";
    document.getElementById("li4").value = "oxtemp1";
  });

  document.getElementById("db4_val2").addEventListener('click', function toggle42(e) {
    e.preventDefault();
    document.getElementById("dv4").innerHTML = "Ox. Temp 2";
    document.getElementById("li4").value = "oxtemp2";
  });

  document.getElementById("db4_val3").addEventListener('click', function toggle43(e) {
    e.preventDefault();
    document.getElementById("dv4").innerHTML = "Pre-Injector Temp";
    document.getElementById("li4").value = "preinjtemp";
  });

  document.getElementById("db4_val4").addEventListener('click', function toggle44(e) {
    e.preventDefault();
    document.getElementById("dv4").innerHTML = "Post-Injector Temp";
    document.getElementById("li4").value = "postinjtemp";
  });

  document.getElementById("db5_val1").addEventListener('click', function toggle51(e) {
    e.preventDefault();
    document.getElementById("dv5").innerHTML = "Ox. Tank Pres";
    document.getElementById("li5").value = "oxpres";
  });

  document.getElementById("db5_val2").addEventListener('click', function toggle52(e) {
    e.preventDefault();
    document.getElementById("dv5").innerHTML = "Pre-Injector Pres";
    document.getElementById("li5").value = "preinjpres";
  });

  document.getElementById("db5_val3").addEventListener('click', function toggle53(e) {
    e.preventDefault();
    document.getElementById("dv5").innerHTML = "Comb. Chamber Pres";
    document.getElementById("li5").value = "ccpres";
  });

  document.getElementById("db6_val1").addEventListener('click', function toggle61(e) {
    e.preventDefault();
    document.getElementById("dv6").innerHTML = "Ox. Tank Pres";
    document.getElementById("li6").value = "oxpres";
  });

  document.getElementById("db6_val2").addEventListener('click', function toggle62(e) {
    e.preventDefault();
    document.getElementById("dv6").innerHTML = "Pre-Injector Pres";
    document.getElementById("li6").value = "preinjpres";
  });

  document.getElementById("db6_val3").addEventListener('click', function toggle63(e) {
    e.preventDefault();
    document.getElementById("dv6").innerHTML = "Comb. Chamber Pres";
    document.getElementById("li6").value = "ccpres";
  });

  document.getElementById("db7_val1").addEventListener('click', function toggle71(e) {
    e.preventDefault();
    document.getElementById("dv7").innerHTML = "Ox. Tank Pres";
    document.getElementById("li7").value = "oxpres";
  });

  document.getElementById("db7_val2").addEventListener('click', function toggle72(e) {
    e.preventDefault();
    document.getElementById("dv7").innerHTML = "Pre-Injector Pres";
    document.getElementById("li7").value = "preinjpres";
  });

  document.getElementById("db7_val3").addEventListener('click', function toggle73(e) {
    e.preventDefault();
    document.getElementById("dv7").innerHTML = "Comb. Chamber Pres";
    document.getElementById("li7").value = "ccpres";
  });

  // pressing buttons

  document.getElementById("btn_dump").addEventListener('click', function statusDump(e) {
    e.preventDefault();
    document.getElementById("status").innerHTML = "Dumping...";
  });

  document.getElementById("btn_fill").addEventListener('click', function statusFill(e) {
    e.preventDefault();
    document.getElementById("status").innerHTML = "Filling...";
  });

  document.getElementById("btn_vent").addEventListener('click', function statusVent(e) {
    e.preventDefault();
    document.getElementById("status").innerHTML = "Venting...";
  });

  document.getElementById("btn_launch").addEventListener('click', function statusLaunch(e) {
    e.preventDefault();
    document.getElementById("status").innerHTML = "Arming Igniter...";
  });

  document.getElementById("btn_abort").addEventListener('click', function statusAbort(e) {
    e.preventDefault();
    document.getElementById("status").innerHTML = "Aborting...";
  });

  document.getElementById("btn_arm_umbilical").addEventListener('click', function statusArm(e) {
    e.preventDefault();
    document.getElementById("status").innerHTML = "Arming Umbilical...";
  });

  document.getElementById("btn_disconnect_umbilical").addEventListener('click', function statusDisconnect(e) {
    e.preventDefault();
    document.getElementById("status").innerHTML = "Disconnecting Umbilical...";
  });

  document.getElementById("btn_submit").addEventListener('click', function statusSubmit(e) {
    e.preventDefault();
    document.getElementById("status").innerHTML = "Submitting Mass...";
  });

}
