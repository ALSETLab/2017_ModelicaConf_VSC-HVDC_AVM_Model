within ;
package VSCHVDC

  package HVDC_Emtp

    model Inductor "Ideal linear electrical inductors"
      extends Modelica.Electrical.MultiPhase.Interfaces.TwoPlug;
      parameter Real i_init_a=34.7058;
      parameter Real i_init_b=-873.189;
      parameter Real i_init_c=838.4833;
      parameter Modelica.SIunits.Inductance L[m](start=fill(1, m)) "Inductance";
      Modelica.Electrical.Analog.Basic.Inductor inductor[m](final L=L, i(start={i_init_a,i_init_b,i_init_c})) annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0)));
    equation
      connect(inductor.p, plug_p.pin) annotation (Line(points={{-10,0},{-100,0}}, color={0,0,255}));
      connect(inductor.n, plug_n.pin) annotation (Line(points={{10,0},{100,0}}, color={0,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
            Ellipse(extent={{-60,-15},{-30,15}}, lineColor={0,0,255}),
            Ellipse(extent={{-30,-15},{0,15}}, lineColor={0,0,255}),
            Ellipse(extent={{0,-15},{30,15}}, lineColor={0,0,255}),
            Ellipse(extent={{30,-15},{60,15}}, lineColor={0,0,255}),
            Rectangle(
              extent={{-60,-30},{60,0}},
              lineColor={255,255,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(points={{60,0},{90,0}}, color={0,0,255}),
            Line(points={{-90,0},{-60,0}}, color={0,0,255}),
            Text(
              extent={{-150,40},{150,100}},
              textString="%name",
              lineColor={0,0,255}),
            Text(
              extent={{-100,-100},{100,-60}},
              lineColor={0,0,0},
              textString="m=%m")}), Documentation(info="<HTML>
<p>
Contains m inductors (Modelica.Electrical.Analog.Basic.Inductor)
</p>
</HTML>"));
    end Inductor;

    package EquivalentSource "This package contains one equivalent source "

      model EqG1 "Multiphase cosine voltage source"
        extends Modelica.Electrical.MultiPhase.Interfaces.TwoPlug;
        parameter Real Uac=400 "Line to Line rms voltage (KV)";
        parameter Real Scc=10000 "Generator short circuit capacity (MVA)";
        parameter Real k=10 "R/L ratio";
        parameter Real f=50 "Frequency, (Hz)";
        parameter Real angle=0 "Initial angle phase a (deg)";
        parameter Real Vm=Uac*(1000/sqrt(3))*sqrt(2);
        //parameter Modelica.SIunits.Voltage V[m](start=fill(1, m)) "Amplitudes of cosine waves";
        parameter Real Phase_a=0;
        parameter Real Phase_b=-120*pi/180;
        parameter Real Phase_c=120*pi/180;
        parameter Real Rd=(Uac*Uac)/(Scc*sqrt(1 + k^2));
        parameter Real Ld=(k*Rd)/(2*pi*f);
        constant Real pi=Modelica.Constants.pi;
        //parameter Modelica.SIunits.Angle phase[m]=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m) "Phases of cosine waves";
        // parameter Modelica.SIunits.Frequency freqHz[m](start=fill(1, m)) "Frequencies of cosine waves";
        parameter Modelica.SIunits.Voltage offset[3]=zeros(3) "Voltage offsets";
        parameter Modelica.SIunits.Time startTime[3]=zeros(3) "Time offsets";
        Modelica.Electrical.Analog.Sources.CosineVoltage cosineVoltage[m](
          final V=fill(Vm, 3),
          final phase={Phase_a,Phase_b,Phase_c},
          final freqHz=fill(f, 3),
          final offset=offset,
          final startTime=startTime) annotation (Placement(transformation(extent={{-14,-10},{6,10}}, rotation=0)));
      equation
        connect(cosineVoltage.n, plug_n.pin) annotation (Line(points={{6,0},{100,0}}, color={0,0,255}));
        connect(cosineVoltage.p, plug_p.pin) annotation (Line(points={{-14,0},{-100,0}}, color={0,0,255}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={Line(points={{-90,0},{-50,0}}, color={0,0,255}),Line(points={{50,0},{90,0}}, color={0,0,255}),
                Ellipse(
                      extent={{-50,50},{50,-50}},
                      lineColor={0,0,255},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid),Line(points={{-50,0},{50,0}}, color={0,0,255}),Text(
                      extent={{-150,-110},{150,-50}},
                      textString="%name",
                      lineColor={0,0,255}),Text(
                      extent={{-100,100},{100,60}},
                      lineColor={0,0,0},
                      textString="m=%m"),Text(
                      extent={{32,60},{112,0}},
                      lineColor={0,0,255},
                      textString="-"),Text(
                      extent={{-110,60},{-30,0}},
                      lineColor={0,0,255},
                      textString="+"),Line(
                      points={{-71,70},{-68.4,69.8},{-63.5,67},{-58.6,61},{-53.6,52},{-48,38.6},{-40.98,18.6},{-26.21,-26.9},{-19.9,-44},{-14.2,-56.2},{-9.3,-64},{-4.4,-68.6},{0.5,-70},{5.5,-67.9},{
                  10.4,-62.5},{15.3,-54.1},{20.9,-41.3},{28,-21.7},{35,0}},
                      color={192,192,192},
                      smooth=Smooth.Bezier),Line(points={{35,0},{44.8,29.9},{51.2,46.5},{56.8,58.1},{61.7,65.2},{66.7,69.2},{71.6,69.8}}, color={192,192,192})}),
          Documentation(info="<HTML>
<p>
Contains m cosine voltage sources (Modelica.Electrical.Analog.Sources.CosineVoltage)
with a default phase shift determined by
<a href=\"modelica://Modelica.Electrical.MultiPhase.Functions.symmetricOrientation\">symmetricOrientation</a>.
</p>
</HTML>"),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
      end EqG1;

      model Equivalent_Source "Equivalent Source reference EMTP-RV"
        parameter Real Uac=400 "Line to Line rms voltage (KV)";
        parameter Real Scc=10000 "Generator short circuit capacity (MVA)";
        parameter Real k=10 "R/L ratio";
        parameter Real f=50 "Frequency, (Hz)";
        parameter Real angle=0 "Initial angle phase a (deg)";
        parameter Real Vm=Uac*(1000/sqrt(3))*sqrt(2);
        //parameter Modelica.SIunits.Voltage V[m](start=fill(1, m)) "Amplitudes of cosine waves";
        parameter Real Phase_a=0;
        parameter Real Phase_b=-120*pi/180;
        parameter Real Phase_c=120*pi/180;
        parameter Real Rd=(Uac*Uac)/(Scc*sqrt(1 + k^2));
        parameter Real Ld=(k*Rd)/(2*pi*f);
        parameter Real ia0=34.7058;
        parameter Real ib0=-873.189;
        parameter Real ic0=838.4833;
        constant Real pi=Modelica.Constants.pi;
        parameter Modelica.SIunits.Voltage offset[3]=zeros(3) "Voltage offsets";
        parameter Modelica.SIunits.Time startTime[3]=zeros(3) "Time offsets";

        Modelica.Electrical.MultiPhase.Basic.Resistor resistor(R=fill(Rd, 3)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={-38,0})));
        Modelica.Electrical.MultiPhase.Interfaces.PositivePlug positivePlug annotation (Placement(transformation(extent={{-114,-10},{-94,10}})));
        Modelica.Electrical.MultiPhase.Interfaces.NegativePlug negativePlug annotation (Placement(transformation(extent={{96,-10},{116,10}})));
        Inductor inductor1(
          L=fill(Ld, 3),
          i_init_a=ia0,
          i_init_b=ib0,
          i_init_c=ic0) annotation (Placement(transformation(extent={{-86,-10},{-66,10}})));
        EqG1 eqG1_1(
          Uac=Uac,
          Scc=Scc,
          k=k,
          f=f,
          angle=angle,
          Vm=Vm,
          Phase_a=Phase_a,
          Phase_b=Phase_b,
          Phase_c=Phase_c,
          Rd=Rd,
          Ld=Ld,
          offset=offset,
          startTime=startTime) annotation (Placement(transformation(extent={{4,-10},{24,10}})));
      equation
        connect(inductor1.plug_n, resistor.plug_n) annotation (Line(points={{-66,0},{-57,0},{-48,0}}, color={0,0,255}));
        connect(inductor1.plug_p, positivePlug) annotation (Line(points={{-86,0},{-104,0}}, color={0,0,255}));
        connect(eqG1_1.plug_n, negativePlug) annotation (Line(points={{24,0},{65,0},{106,0}}, color={0,0,255}));
        connect(eqG1_1.plug_p, resistor.plug_p) annotation (Line(points={{4,0},{-12,0},{-28,0}}, color={0,0,255}));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})), Icon(
              graphics={
                Ellipse(
                      extent={{-40,54},{60,-46}},
                      lineColor={0,0,255},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid),
              Line(points={{60,0},{96,0}}, color={28,108,200}),
              Line(points={{-94,0},{-40,0}}, color={28,108,200})}));
      end Equivalent_Source;

      model Equivalent_Source2
        parameter Real Uac=400 "Line to Line rms voltage (KV)";
        parameter Real Scc=10000 "Generator short circuit capacity (MVA)";
        parameter Real k=10 "R/L ratio";
        parameter Real f=50 "Frequency, (Hz)";
        parameter Real angle=0 "Initial angle phase a (deg)";
        parameter Real Vm=Uac*(1000/sqrt(3))*sqrt(2);
        //parameter Modelica.SIunits.Voltage V[m](start=fill(1, m)) "Amplitudes of cosine waves";
        parameter Real Phase_a=0;
        parameter Real Phase_b=-120*pi/180;
        parameter Real Phase_c=120*pi/180;
        parameter Real Rd=(Uac*Uac)/(Scc*sqrt(1 + k^2));
        parameter Real Ld=(k*Rd)/(2*pi*f);
        parameter Real ia0=34.7058;
        parameter Real ib0=-873.189;
        parameter Real ic0=838.4833;
        constant Real pi=Modelica.Constants.pi;
        parameter Modelica.SIunits.Voltage offset[3]=zeros(3) "Voltage offsets";
        parameter Modelica.SIunits.Time startTime[3]=zeros(3) "Time offsets";

        Modelica.Electrical.MultiPhase.Interfaces.PositivePlug positivePlug annotation (Placement(transformation(extent={{-114,-10},{-94,10}})));
        Modelica.Electrical.MultiPhase.Interfaces.NegativePlug negativePlug annotation (Placement(transformation(extent={{96,-10},{116,10}})));
        EqG1 eqG1_1(
          Uac=Uac,
          Scc=Scc,
          k=k,
          f=f,
          angle=angle,
          Vm=Vm,
          Phase_a=Phase_a,
          Phase_b=Phase_b,
          Phase_c=Phase_c,
          Rd=Rd,
          Ld=Ld,
          offset=offset,
          startTime=startTime) annotation (Placement(transformation(extent={{4,-10},{24,10}})));
      equation
        connect(eqG1_1.plug_n, negativePlug) annotation (Line(points={{24,0},{65,0},{106,0}}, color={0,0,255}));
        connect(positivePlug, eqG1_1.plug_p) annotation (Line(points={{-104,0},{4,0},{4,0}}, color={0,0,255}));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
      end Equivalent_Source2;
    end EquivalentSource;

    model ThreePhsaeYDtransformer
      "Three Phase Transformer model, Reference EMTP-RV"
      parameter Real R_trans_prim=0.144 "(Ohm)";
      parameter Real L_trans_prim=0.08250592249883855 "(H)";
      parameter Real R_trans_secon=0.010239999999999999 "(Ohm)";
      parameter Real L_trans_secon=0.005867087822139628 "(H)";
      parameter Real Vac_primary=400 "(KV)";
      parameter Real Vac_secondary=320 "(KV)";
      //parameter Real trans_ratio=Vac_secondary/Vac_primary;
      parameter Real trans_ratio=(Vac_primary/Vac_secondary)/sqrt(3);
      Modelica.Electrical.MultiPhase.Interfaces.PositivePlug positivePlug annotation (Placement(transformation(extent={{-114,-10},{-94,10}})));
      Modelica.Electrical.MultiPhase.Interfaces.NegativePlug negativePlug annotation (Placement(transformation(extent={{94,-10},{114,10}})));
      RL rL(R=R_trans_prim, L=L_trans_prim) annotation (Placement(transformation(extent={{-56,38},{-36,58}})));
      RL rL1(R=R_trans_prim, L=L_trans_prim) annotation (Placement(transformation(extent={{-56,6},{-36,26}})));
      RL rL2(R=R_trans_prim, L=L_trans_prim) annotation (Placement(transformation(extent={{-56,-30},{-36,-10}})));
      Modelica.Electrical.Analog.Ideal.IdealTransformer idealTransformer(n=trans_ratio) annotation (Placement(transformation(extent={{-10,36},{10,56}})));
      Modelica.Electrical.Analog.Ideal.IdealTransformer idealTransformer1(n=trans_ratio, considerMagnetization=false) annotation (Placement(transformation(extent={{-10,2},{10,22}})));
      Modelica.Electrical.Analog.Ideal.IdealTransformer idealTransformer2(n=trans_ratio) annotation (Placement(transformation(extent={{-8,-36},{12,-16}})));
      Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(transformation(extent={{-32,-62},{-12,-42}})));
      Modelica.Electrical.Analog.Basic.Ground ground1 annotation (Placement(transformation(extent={{-32,-20},{-12,0}})));
      Modelica.Electrical.Analog.Basic.Ground ground2 annotation (Placement(transformation(extent={{-34,18},{-14,38}})));
      RL rL3(
        R=R_trans_secon,
        L=L_trans_secon,
        iStart=875.053) annotation (Placement(transformation(extent={{38,42},{58,62}})));
      RL rL4(
        R=R_trans_secon,
        L=L_trans_secon,
        iStart=-1488.26) annotation (Placement(transformation(extent={{38,8},{58,28}})));
      RL rL5(
        R=R_trans_secon,
        L=L_trans_secon,
        iStart=613.228) annotation (Placement(transformation(extent={{38,-28},{58,-8}})));
    equation
      connect(positivePlug.pin[2], rL1.pin_p) annotation (Line(points={{-104,0},{-90,0},{-90,16},{-56.6,16}}, color={0,0,255}));
      connect(positivePlug.pin[3], rL2.pin_p) annotation (Line(points={{-104,0},{-90,0},{-90,-20},{-56.6,-20}}, color={0,0,255}));
      connect(rL.pin_n, idealTransformer.p1) annotation (Line(points={{-35.8,48},{-24,48},{-24,51},{-10,51}}, color={0,0,255}));
      connect(rL1.pin_n, idealTransformer1.p1) annotation (Line(points={{-35.8,16},{-28,16},{-28,18},{-10,18},{-10,17}}, color={0,0,255}));
      connect(rL2.pin_n, idealTransformer2.p1) annotation (Line(points={{-35.8,-20},{-22,-20},{-22,-21},{-8,-21}}, color={0,0,255}));
      connect(idealTransformer.n1, ground2.p) annotation (Line(points={{-10,41},{-18,41},{-18,38},{-24,38}}, color={0,0,255}));
      connect(idealTransformer1.n1, ground1.p) annotation (Line(points={{-10,7},{-22,7},{-22,0}}, color={0,0,255}));
      connect(idealTransformer2.n1, ground.p) annotation (Line(points={{-8,-31},{-22,-31},{-22,-42}}, color={0,0,255}));
      connect(idealTransformer.p2, rL3.pin_p) annotation (Line(points={{10,51},{22,51},{22,52},{37.4,52}},color={0,0,255}));
      connect(idealTransformer.n2, rL4.pin_n) annotation (Line(points={{10,41},{64,41},{64,18},{58.2,18}},color={0,0,255}));
      connect(idealTransformer1.p2, rL4.pin_p) annotation (Line(points={{10,17},{24,17},{24,18},{37.4,18}}, color={0,0,255}));
      connect(idealTransformer1.n2, rL5.pin_n) annotation (Line(points={{10,7},{38,7},{38,6},{62,6},{62,-18},{58.2,-18}}, color={0,0,255}));
      connect(rL5.pin_p, idealTransformer2.p2) annotation (Line(points={{37.4,-18},{12,-18},{12,-21}}, color={0,0,255}));
      connect(rL3.pin_n, negativePlug.pin[1]) annotation (Line(points={{58.2,52},{88,52},{88,0},{104,0}}, color={0,0,255}));
      connect(positivePlug.pin[1], rL.pin_p) annotation (Line(points={{-104,0},{-98,0},{-90,0},{-90,48},{-56.6,48}}, color={0,0,255}));
      connect(rL5.pin_n, negativePlug.pin[3]) annotation (Line(points={{58.2,-18},{104,-18},{104,0}}, color={0,0,255}));
      connect(rL4.pin_n, negativePlug.pin[2]) annotation (Line(points={{58.2,18},{70,18},{70,16},{76,16},{76,0},{104,0}}, color={0,0,255}));
      connect(idealTransformer2.n2, rL3.pin_n) annotation (Line(points={{12,-31},{78,-31},{78,52},{58.2,52}}, color={0,0,255}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end ThreePhsaeYDtransformer;

    model RL
      parameter Real R "Resistance (Ohm)";
      parameter Real L "Inductance (H)";
      parameter Real iStart=0 "Start Value of the inductance_Current";

      Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (Placement(transformation(extent={{-32,-10},{-12,10}})));
      Modelica.Electrical.Analog.Basic.Inductor inductor(i(start=iStart), L=L) annotation (Placement(transformation(extent={{14,-10},{34,10}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation (Placement(transformation(extent={{-116,-10},{-96,10}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation (Placement(transformation(extent={{92,-10},{112,10}})));
    equation
      connect(inductor.n, pin_n) annotation (Line(points={{34,0},{102,0}}, color={0,0,255}));
      connect(resistor.n, inductor.p) annotation (Line(points={{-12,0},{1,0},{14,0}}, color={0,0,255}));
      connect(pin_p, resistor.p) annotation (Line(points={{-106,0},{-32,0}}, color={0,0,255}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end RL;

    model AC_side

      Modelica.Blocks.Interfaces.RealInput vdc annotation (Placement(transformation(extent={{-144,14},{-104,54}})));
      Modelica.Blocks.Interfaces.RealInput vref annotation (Placement(transformation(extent={{-144,-48},{-104,-8}})));
      Modelica.Blocks.Interfaces.RealOutput vac annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    equation
      vac = (vdc*vref)/2;

      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end AC_side;

    model DC_side

      Modelica.Blocks.Interfaces.RealInput vref_Pha annotation (Placement(transformation(extent={{-132,62},{-102,92}})));
      Modelica.Blocks.Interfaces.RealInput vref_Phb annotation (Placement(transformation(extent={{-132,32},{-102,62}})));
      Modelica.Blocks.Interfaces.RealInput vref_Phc annotation (Placement(transformation(extent={{-132,6},{-104,34}})));
      Modelica.Blocks.Interfaces.RealInput Iac_Pha annotation (Placement(transformation(extent={{-132,-24},{-104,4}})));
      Modelica.Blocks.Interfaces.RealInput Iac_Phb annotation (Placement(transformation(extent={{-134,-54},{-106,-26}})));
      Modelica.Blocks.Interfaces.RealInput Iac_Phc annotation (Placement(transformation(extent={{-134,-84},{-106,-56}})));
      Modelica.Blocks.Interfaces.RealOutput I_dc annotation (Placement(transformation(extent={{102,-8},{122,12}})));
    equation
      I_dc = (vref_Pha*Iac_Pha + vref_Phb*Iac_Phb + vref_Phc*Iac_Phc)/2;
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end DC_side;

    model AVM
      parameter Real Lac "(H)";
      parameter Real Rdc "ohm";
      parameter Real Ldc "H";
      parameter Real Ceq "F";
      parameter Real V_C_init;
      parameter Real I_L_init;
      Modelica.Electrical.MultiPhase.Interfaces.PositivePlug AC annotation (Placement(transformation(extent={{-114,-8},{-94,12}})));
      Modelica.Electrical.Analog.Sources.SignalVoltage AC_a annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=270,
            origin={-70,-42})));
      Modelica.Electrical.Analog.Sources.SignalVoltage AC_b annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=270,
            origin={-46,-42})));
      Modelica.Electrical.Analog.Sources.SignalVoltage AC_c annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=270,
            origin={-20,-42})));
      Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(transformation(extent={{-54,-86},{-38,-70}})));
      AC_side AC_side_Pha annotation (Placement(transformation(extent={{-102,-40},{-88,-26}})));
      AC_side AC_side_Phb annotation (Placement(transformation(extent={{-102,-62},{-88,-48}})));
      AC_side AC_side_Phc annotation (Placement(transformation(extent={{-102,-82},{-88,-68}})));
      Modelica.Electrical.Analog.Basic.Inductor L1(L=Lac) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-70,-10})));
      Modelica.Electrical.Analog.Basic.Inductor L2(L=Lac) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-46,-10})));
      Modelica.Electrical.Analog.Basic.Inductor L3(L=Lac) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-20,-10})));
      Modelica.Electrical.MultiPhase.Sensors.CurrentSensor currentSensor annotation (Placement(transformation(extent={{-82,24},{-62,44}})));
      Modelica.Electrical.MultiPhase.Interfaces.NegativePlug AC2;
      DC_side dC_side annotation (Placement(transformation(extent={{20,-22},{40,-2}})));
      Modelica.Electrical.Analog.Sources.SignalCurrent I annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={60,-12})));
      Modelica.Electrical.Analog.Interfaces.PositivePin P annotation (Placement(transformation(extent={{94,46},{114,66}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin N annotation (Placement(transformation(extent={{94,-58},{114,-38}})));
      Modelica.Electrical.Analog.Basic.Resistor resistor(R=Rdc) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={56,24})));
      Modelica.Electrical.Analog.Basic.Inductor inductor(L=Ldc, i(start=I_L_init)) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={56,54})));
      Modelica.Electrical.Analog.Basic.Capacitor capacitor(C=Ceq, v(start=V_C_init)) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={92,-12})));
      Modelica.Blocks.Interfaces.RealInput varef annotation (Placement(transformation(extent={{-146,-46},{-128,-28}})));
      Modelica.Blocks.Interfaces.RealInput vbref annotation (Placement(transformation(extent={{-146,-66},{-130,-50}})));
      Modelica.Blocks.Interfaces.RealInput vcref annotation (Placement(transformation(extent={{-146,-86},{-130,-70}})));
      Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={108,12})));
      Modelica.Electrical.Analog.Semiconductors.Diode diode(Vt=0.7) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={74,-12})));
      Modelica.Blocks.Interfaces.RealOutput VDC annotation (Placement(transformation(extent={{100,-84},{120,-64}})));
    equation
      connect(AC_side_Pha.vac, AC_a.v) annotation (Line(points={{-87.3,-33},{-60,-33},{-60,-42},{-65.8,-42}}, color={0,0,127}));
      connect(AC_side_Phb.vac, AC_b.v) annotation (Line(points={{-87.3,-55},{-80,-55},{-80,-28},{-38,-28},{-38,-42},{-41.8,-42}}, color={0,0,127}));
      connect(AC_side_Phc.vac, AC_c.v) annotation (Line(points={{-87.3,-75},{-82,-75},{-82,-24},{-8,-24},{-8,-42},{-15.8,-42}}, color={0,0,127}));
      connect(L1.p, AC_a.p) annotation (Line(points={{-70,-20},{-70,-36}}, color={0,0,255}));
      connect(L2.p, AC_b.p) annotation (Line(points={{-46,-20},{-46,-20},{-46,-36}}, color={0,0,255}));
      connect(L3.p, AC_c.p) annotation (Line(points={{-20,-20},{-20,-20},{-20,-36}}, color={0,0,255}));
      connect(AC_a.n, ground.p) annotation (Line(points={{-70,-48},{-70,-48},{-70,-66},{-46,-66},{-46,-70}}, color={0,0,255}));
      connect(AC_c.n, ground.p) annotation (Line(points={{-20,-48},{-20,-66},{-46,-66},{-46,-70}}, color={0,0,255}));
      connect(AC_b.n, ground.p) annotation (Line(points={{-46,-48},{-46,-70}}, color={0,0,255}));
      connect(AC, currentSensor.plug_p) annotation (Line(points={{-104,2},{-90,2},{-90,34},{-82,34}}, color={0,0,255}));
      connect(currentSensor.plug_n, AC2) annotation (Line(points={{-62,34},{-56,34},{-48,34}}, color={0,0,255}));
      connect(L1.n, AC2.pin[1]) annotation (Line(points={{-70,0},{-70,14},{-48,14},{-48,34}}, color={0,0,255}));
      connect(AC2.pin[2], L2.n) annotation (Line(points={{-48,34},{-44,34},{-36,34},{-36,0},{-46,0}}, color={0,0,255}));
      connect(AC2.pin[3], L3.n) annotation (Line(points={{-48,34},{-20,34},{-20,0}}, color={0,0,255}));
      connect(currentSensor.i[1], dC_side.Iac_Pha) annotation (Line(points={{-72,
              23.6667},{-72,16},{10,16},{10,-13},{18.2,-13}},                                                                    color={0,0,127}));
      connect(currentSensor.i[2], dC_side.Iac_Phb) annotation (Line(points={{-72,23},{-72,23},{-72,16},{4,16},{4,-16},{18,-16}}, color={0,0,127}));
      connect(currentSensor.i[3], dC_side.Iac_Phc) annotation (Line(points={{-72,
              22.3333},{-72,22.3333},{-72,16},{-8,16},{-8,-20},{18,-20},{18,-19}},                                                                    color={0,0,127}));
      connect(resistor.n, inductor.p) annotation (Line(points={{56,34},{56,44}}, color={0,0,255}));
      connect(inductor.n, P) annotation (Line(points={{56,64},{86,64},{86,56},{104,56}}, color={0,0,255}));
      connect(AC_side_Phb.vdc, AC_side_Phc.vdc) annotation (Line(points={{-103.68,-52.62},{-112,-52.62},{-112,-72},{-103.68,-72.62}}, color={0,0,127}));
      connect(AC_side_Pha.vdc, AC_side_Phc.vdc) annotation (Line(points={{-103.68,-30.62},{-112,-30.62},{-112,-72},{-103.68,-72.62}}, color={0,0,127}));
      connect(AC_side_Pha.vref, dC_side.vref_Pha);
      connect(AC_side_Phb.vref, dC_side.vref_Phb);
      connect(AC_side_Phc.vref, dC_side.vref_Phc);
      connect(varef, AC_side_Pha.vref) annotation (Line(points={{-137,-37},{-120.5,-37},{-120.5,-34.96},{-103.68,-34.96}}, color={0,0,127}));
      connect(vbref, AC_side_Phb.vref) annotation (Line(points={{-138,-58},{-103.68,-58},{-103.68,-56.96}}, color={0,0,127}));
      connect(vcref, AC_side_Phc.vref) annotation (Line(points={{-138,-78},{-103.68,-78},{-103.68,-76.96}}, color={0,0,127}));
      connect(P, voltageSensor.p) annotation (Line(points={{104,56},{106,56},{106,22},{108,22}}, color={0,0,255}));
      connect(voltageSensor.n, N) annotation (Line(points={{108,2},{108,2},{108,-48},{104,-48}}, color={0,0,255}));
      connect(voltageSensor.v, AC_side_Phc.vdc)
        annotation (Line(points={{118,12},{122,12},{122,8},{124,8},{124,-96},{-118,-96},{-118,-64},{-114,-64},{-114,-72.62},{-103.68,-72.62}}, color={0,0,127}));
      connect(voltageSensor.v, VDC) annotation (Line(points={{118,12},{134,12},{134,-62},{86,-62},{86,-74},{110,-74}}, color={0,0,127}));
      connect(dC_side.I_dc, I.i) annotation (Line(points={{41.2,-11.8},{46.6,-11.8},{46.6,-12},{53,-12}}, color={0,0,127}));
      connect(I.p, N) annotation (Line(points={{60,-22},{60,-48},{104,-48}}, color={0,0,255}));
      connect(I.p, diode.p) annotation (Line(points={{60,-22},{74,-22}}, color={0,0,255}));
      connect(diode.p, capacitor.p) annotation (Line(points={{74,-22},{92,-22}}, color={0,0,255}));
      connect(I.n, diode.n) annotation (Line(points={{60,-2},{67,-2},{74,-2}}, color={0,0,255}));
      connect(capacitor.n, diode.n) annotation (Line(points={{92,-2},{74,-2}}, color={0,0,255}));
      connect(resistor.p, I.n) annotation (Line(points={{56,14},{56,-2},{60,-2}}, color={0,0,255}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end AVM;

    model Clark

      Modelica.Blocks.Interfaces.RealInput V_abc_Y[3]
        "Va, Vb and Vc in the primary side of Transformar"                                               annotation (Placement(transformation(extent={{-146,60},{-106,100}})));
      Modelica.Blocks.Interfaces.RealInput I_abc_Y[3]
        "Ia, Ib and Ic in the primary side of Transformar"                                               annotation (Placement(transformation(extent={{-146,12},{-106,52}})));
      Modelica.Blocks.Interfaces.RealInput V_abc_D[3] annotation (Placement(transformation(extent={{-146,-50},{-106,-10}})));
      Modelica.Blocks.Interfaces.RealInput I_abc_D[3] annotation (Placement(transformation(extent={{-146,-98},{-106,-58}})));
      Modelica.Blocks.Interfaces.RealOutput V_ab_Y[2] annotation (Placement(transformation(extent={{100,70},{120,90}})));
      Modelica.Blocks.Interfaces.RealOutput I_ab_Y[2] annotation (Placement(transformation(extent={{100,20},{120,40}})));
      Modelica.Blocks.Interfaces.RealOutput V_ab_D[2] annotation (Placement(transformation(extent={{100,-34},{120,-14}})));
      //
      Modelica.Blocks.Interfaces.RealOutput I_ab_D[2] annotation (Placement(transformation(extent={{100,-84},{120,-64}})));
    equation

      V_ab_Y[1] = 1/sqrt(3)*(V_abc_Y[1] - V_abc_Y[3]);
      V_ab_Y[2] = -(1/3)*V_abc_Y[1] + (2/3)*V_abc_Y[2] - (1/3)*V_abc_Y[3];
      I_ab_Y[1] = 1/sqrt(3)*(I_abc_Y[1] - I_abc_Y[3]);
      I_ab_Y[2] = -(1/3)*I_abc_Y[1] + (2/3)*I_abc_Y[2] - (1/3)*I_abc_Y[3];
      V_ab_D[1] = (2/3)*V_abc_D[1] - (1/3)*V_abc_D[2] - (1/3)*V_abc_D[3];
      V_ab_D[2] = 1/sqrt(3)*(V_abc_D[2] - V_abc_D[3]);
      I_ab_D[1] = (2/3)*I_abc_D[1] - (1/3)*I_abc_D[2] - (1/3)*I_abc_D[3];
      I_ab_D[2] = 1/sqrt(3)*(I_abc_D[2] - I_abc_D[3]);

      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end Clark;

    model PI
      parameter Real init_value;
      parameter Real max;
      parameter Real min;
      parameter Real KI;
      parameter Real KP;
      parameter Real time_step=0.001;

      Modelica.Blocks.Continuous.LimIntegrator limIntegrator(
        k=1,
        outMax=1e5,
        initType=Modelica.Blocks.Types.Init.InitialOutput,
        y_start=init_value,
        limitsAtInit=true) annotation (Placement(transformation(extent={{-14,-10},{6,10}})));
      Modelica.Blocks.Interfaces.RealInput u annotation (Placement(transformation(extent={{-142,-20},{-102,20}})));
      Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Modelica.Blocks.Math.Add add(k2=+1) annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
      Modelica.Blocks.Math.Add add1(k1=+1, k2=-1) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={42,-38})));
      Modelica.Blocks.Math.Add add2(k1=KP, k2=KI) annotation (Placement(transformation(extent={{26,18},{42,34}})));
      Modelica.Blocks.Nonlinear.Limiter limiter(uMax=max, uMin=min) annotation (Placement(transformation(extent={{58,20},{74,36}})));
    equation
      connect(add.y, limIntegrator.u) annotation (Line(points={{-49,0},{-16,0}}, color={0,0,127}));
      connect(u, add.u1) annotation (Line(points={{-122,0},{-106,0},{-92,0},{-92,6},{-72,6}}, color={0,0,127}));
      connect(add2.u1, add.u1) annotation (Line(points={{24.4,30.8},{-92,30.8},{-92,6},{-72,6}}, color={0,0,127}));
      connect(limIntegrator.y, add2.u2) annotation (Line(points={{7,0},{16,0},{16,21.2},{24.4,21.2}}, color={0,0,127}));
      connect(add2.y, limiter.u) annotation (Line(points={{42.8,26},{48,26},{48,28},{56.4,28}}, color={0,0,127}));
      connect(limiter.y, y) annotation (Line(points={{74.8,28},{88,28},{88,0},{110,0}}, color={0,0,127}));
      connect(add1.u2, limiter.u) annotation (Line(points={{54,-32},{58,-32},{58,-10},{48,-10},{48,28},{56.4,28}}, color={0,0,127}));
      connect(add1.u1, y) annotation (Line(points={{54,-44},{92,-44},{92,0},{110,0}}, color={0,0,127}));
      connect(add1.y, add.u2) annotation (Line(points={{31,-38},{-26,-38},{-26,-40},{-88,-40},{-88,-6},{-72,-6}}, color={0,0,127}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end PI;

    model P_Controller
      parameter Real KIP "Integral Gain of Active Power Regulator";
      parameter Real KPP "Proportional gain of Active Power Regulator";
      parameter Real P_ref "Active power reference (p.u.)";
      parameter Real P_step
        "Step (Height) value of Active Power, in the reference";
      parameter Real start_time_P "Start Time of the step";
      parameter Real init_P "Initial Output of Integrator of P controller";
      parameter Real P_Vdc_Kp "Gain of Deadband control";
      deadband deadband1(KP_DB=P_Vdc_Kp) annotation (Placement(transformation(extent={{-4,22},{16,42}})));
      Modelica.Blocks.Sources.Step step(
        height=P_step,
        offset=P_ref,
        startTime=start_time_P) annotation (Placement(transformation(extent={{-96,8},{-76,28}})));
      Modelica.Blocks.Interfaces.RealInput P annotation (Placement(transformation(extent={{-146,-46},{-106,-6}})));
      Modelica.Blocks.Nonlinear.Limiter limiter(uMax=1.2) annotation (Placement(transformation(extent={{-58,10},{-42,26}})));
      Modelica.Blocks.Math.Add add(k2=-1) annotation (Placement(transformation(extent={{-24,-30},{-4,-10}})));
      PI pI(
        init_value=init_P,
        max=1.2,
        min=-1.2,
        KI=KIP,
        KP=KPP) annotation (Placement(transformation(extent={{18,-30},{38,-10}})));
      Modelica.Blocks.Interfaces.RealInput Vdc annotation (Placement(transformation(extent={{-148,26},{-108,66}})));
      Modelica.Blocks.Math.Add add1(k2=+1) annotation (Placement(transformation(extent={{60,-24},{80,-4}})));
      Modelica.Blocks.Interfaces.RealOutput PREF annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    equation

      connect(step.y, limiter.u) annotation (Line(points={{-75,18},{-59.6,18}}, color={0,0,127}));
      connect(P, add.u2) annotation (Line(points={{-126,-26},{-26,-26}}, color={0,0,127}));
      connect(limiter.y, add.u1) annotation (Line(points={{-41.2,18},{-34,18},{-34,-14},{-26,-14}}, color={0,0,127}));
      connect(add.y, pI.u) annotation (Line(points={{-3,-20},{15.8,-20}}, color={0,0,127}));
      connect(Vdc, deadband1.u) annotation (Line(points={{-128,46},{-84,46},{-40,46},{-40,32},{-6.4,32}}, color={0,0,127}));
      connect(pI.y, add1.u2) annotation (Line(points={{39,-20},{48.5,-20},{58,-20}}, color={0,0,127}));
      connect(deadband1.y, add1.u1) annotation (Line(points={{17,32},{46,32},{46,-8},{58,-8}}, color={0,0,127}));
      connect(add1.y, PREF) annotation (Line(points={{81,-14},{88,-14},{88,0},{110,0}}, color={0,0,127}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end P_Controller;

    model deadband

      parameter Real Vdcmax=1.05;
      parameter Real Vdcmin=0.95;
      parameter Real KP_DB;
      Modelica.Blocks.Interfaces.RealInput u annotation (Placement(transformation(extent={{-144,-20},{-104,20}})));
      Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    equation

      if u > Vdcmax then
        y = (Vdcmax - u)*KP_DB;
      elseif u < Vdcmin then
        y = (Vdcmin - u)*KP_DB;
      else
        y = 0;
      end if;
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})), Icon(graphics={Rectangle(
              rotation=0,
              lineColor={0,0,255},
              fillColor={0,0,255},
              pattern=LinePattern.Solid,
              fillPattern=FillPattern.None,
              lineThickness=0.25,
              extent={{-66.787,67.148},{61.7329,-47.6534}}), Text(
              rotation=0,
              lineColor={0,0,255},
              fillColor={0,0,255},
              pattern=LinePattern.Solid,
              fillPattern=FillPattern.None,
              lineThickness=0.25,
              extent={{-51.6245,38.6282},{43.6823,-9.74729}},
              textString="deadband")}));
    end deadband;

    model VDC_Controller

      parameter Real VDC_ref "VDC reference value";
      parameter Real KIVDC "Integral gain of VDC controller";
      parameter Real KPVDC "Proportional gain of VDC controller";
      parameter Real init_value_OVdc
        "Initial Output of Integrator of VDC controller";
      parameter Real Vdc_step "Step (Height) value of VDC in the reference";
      parameter Real t_VdcStep "Start Time of the step";

      Modelica.Blocks.Sources.Step step(
        height=Vdc_step,
        offset=VDC_ref,
        startTime=t_VdcStep) annotation (Placement(transformation(extent={{-98,48},{-78,68}})));
      Modelica.Blocks.Interfaces.RealInput u annotation (Placement(transformation(extent={{-142,-20},{-102,20}})));
      Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      HVDC_Emtp.PI pI(
        init_value=init_value_OVdc,
        max=1.2,
        min=-1.2,
        KI=KIVDC,
        KP=KPVDC) annotation (Placement(transformation(extent={{-18,-10},{10,18}})));
      Modelica.Blocks.Math.Add add(k2=-1) annotation (Placement(transformation(extent={{-60,-6},{-40,14}})));
    equation
      connect(step.y, add.u1) annotation (Line(points={{-77,58},{-68,58},{-68,10},{-62,10}}, color={0,0,127}));
      connect(u, add.u2) annotation (Line(points={{-122,0},{-80,0},{-80,-2},{-62,-2}}, color={0,0,127}));
      connect(add.y, pI.u) annotation (Line(points={{-39,4},{-28,4},{-21.08,4}}, color={0,0,127}));
      connect(pI.y, y) annotation (Line(points={{11.4,4},{52,4},{94,4},{94,0},{110,0}}, color={0,0,127}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end VDC_Controller;

    model Q_Controller
      parameter Real KIQ "Integral gain of Q controller";
      parameter Real KPQ "Proportional gain of P controller";
      parameter Real Q_ref "Reference value of Reactive Power (p.u.)";
      parameter Real Q_step
        "Step (Height) value of Reactive Power, in the reference";
      parameter Real Qinit_value "Initial Output of Integrator of Q controller";
      parameter Real start_time_Q "Start Time of the step";
      Modelica.Blocks.Sources.Step step(
        height=Q_step,
        offset=Q_ref,
        startTime=start_time_Q) annotation (Placement(transformation(extent={{-96,34},{-76,54}})));
      Modelica.Blocks.Interfaces.RealInput Q annotation (Placement(transformation(extent={{-142,-32},{-102,8}})));
      Modelica.Blocks.Math.Add add(k2=-1) annotation (Placement(transformation(extent={{-44,-10},{-24,10}})));
      PI pI(
        init_value=Qinit_value,
        max=0.5,
        min=-0.5,
        KI=KIQ,
        KP=KPQ) annotation (Placement(transformation(extent={{4,-10},{24,10}})));
      Modelica.Blocks.Interfaces.RealOutput QREF annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Modelica.Blocks.Math.Gain gain(k=-1) annotation (Placement(transformation(extent={{44,-10},{64,10}})));
    equation

      connect(step.y, add.u1) annotation (Line(points={{-75,44},{-58,44},{-58,6},{-46,6}}, color={0,0,127}));
      connect(add.y, pI.u) annotation (Line(points={{-23,0},{1.8,0}}, color={0,0,127}));
      connect(Q, add.u2) annotation (Line(points={{-122,-12},{-82,-12},{-82,-6},{-46,-6}}, color={0,0,127}));
      connect(pI.y, gain.u) annotation (Line(points={{25,0},{42,0}}, color={0,0,127}));
      connect(gain.y, QREF) annotation (Line(points={{65,0},{110,0},{110,0}}, color={0,0,127}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end Q_Controller;

    model VAC_Controller
      parameter Real Vac_ref "Reference AC voltage (p.u.)";
      parameter Real Vac_step "Step (Height) value of VAC, in the reference";
      parameter Real Vac_init "Initial Output of Integrator of VAC controller";
      parameter Real Vac_step_time "Start Time of the step";
      parameter Real KIVAC "Integral Gain of VAC controller";
      parameter Real KPVAC "Proportional Gain of VAC contoller";
      Modelica.Blocks.Sources.Step step(
        height=Vac_step,
        offset=Vac_ref,
        startTime=Vac_step_time) annotation (Placement(transformation(extent={{-94,34},{-74,54}})));
      Modelica.Blocks.Math.Add add(k2=-1) annotation (Placement(transformation(extent={{-42,-10},{-22,10}})));
      PI pI(
        max=0.5,
        min=-0.5,
        init_value=Vac_step_time,
        KI=KIVAC,
        KP=KPVAC) annotation (Placement(transformation(extent={{6,-10},{26,10}})));
      Modelica.Blocks.Math.Gain gain(k=-1) annotation (Placement(transformation(extent={{-62,-68},{-42,-48}})));
      Modelica.Blocks.Interfaces.RealInput VAC annotation (Placement(transformation(extent={{-140,-32},{-100,8}})));
      Modelica.Blocks.Interfaces.RealOutput Idref annotation (Placement(transformation(extent={{102,-10},{122,10}})));
      Modelica.Blocks.Interfaces.RealInput Iq annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
      Modelica.Blocks.Math.Add add1(k2=-1) annotation (Placement(transformation(extent={{46,-10},{66,10}})));
    equation

      connect(step.y, add.u1) annotation (Line(points={{-73,44},{-56,44},{-56,6},{-44,6}}, color={0,0,127}));
      connect(add.y, pI.u) annotation (Line(points={{-21,0},{3.8,0}}, color={0,0,127}));
      connect(VAC, add.u2) annotation (Line(points={{-120,-12},{-80,-12},{-80,-6},{-44,-6}}, color={0,0,127}));
      connect(pI.y, add1.u1) annotation (Line(points={{27,0},{32,0},{32,6},{44,6}}, color={0,0,127}));
      connect(Iq, gain.u) annotation (Line(points={{-120,-60},{-94,-60},{-94,-58},{-64,-58}}, color={0,0,127}));
      connect(gain.y, add1.u2) annotation (Line(points={{-41,-58},{-2,-58},{32,-58},{32,-6},{44,-6}}, color={0,0,127}));
      connect(add1.y, Idref) annotation (Line(points={{67,0},{112,0}}, color={0,0,127}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end VAC_Controller;

    model Power_Current

      Modelica.Blocks.Interfaces.RealInput P_REF annotation (Placement(transformation(extent={{-150,50},{-110,90}})));
      Modelica.Blocks.Interfaces.RealInput Q_REF annotation (Placement(transformation(extent={{-150,-8},{-110,32}})));
      Modelica.Blocks.Interfaces.RealInput VacGrid annotation (Placement(transformation(extent={{-150,-64},{-110,-24}})));
      Modelica.Blocks.Nonlinear.Limiter limiter(uMin=0.01, uMax=2) annotation (Placement(transformation(extent={{-30,-52},{-14,-36}})));
      Modelica.Blocks.Math.Division division annotation (Placement(transformation(extent={{-20,60},{0,80}})));
      Modelica.Blocks.Math.Division division1 annotation (Placement(transformation(extent={{-22,4},{-2,24}})));
      Modelica.Blocks.Interfaces.RealOutput Id_Ref annotation (Placement(transformation(extent={{100,26},{120,46}})));
      Modelica.Blocks.Interfaces.RealOutput Iq_Ref annotation (Placement(transformation(extent={{100,-38},{120,-18}})));
    equation
      connect(limiter.y, division.u2) annotation (Line(points={{-13.2,-44},{14,-44},{14,36},{-22,36},{-22,64}}, color={0,0,127}));
      connect(P_REF, division.u1) annotation (Line(points={{-130,70},{-70,70},{-70,76},{-22,76}}, color={0,0,127}));
      connect(limiter.y, division1.u2) annotation (Line(points={{-13.2,-44},{0,-44},{0,-12},{-38,-12},{-38,8},{-24,8}}, color={0,0,127}));
      connect(Q_REF, division1.u1) annotation (Line(points={{-130,12},{-100,12},{-100,8},{-42,8},{-42,20},{-24,20}}, color={0,0,127}));
      connect(division.y, Id_Ref) annotation (Line(points={{1,70},{22,70},{56,70},{56,36},{110,36}}, color={0,0,127}));
      connect(division1.y, Iq_Ref) annotation (Line(points={{-1,14},{46,14},{46,-28},{110,-28}}, color={0,0,127}));
      connect(VacGrid, limiter.u) annotation (Line(points={{-130,-44},{-31.6,-44},{-31.6,-44}}, color={0,0,127}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end Power_Current;

    model InnerControl
      parameter Real Ictrl_KP;
      parameter Real Ictrl_KI;
      parameter Real Lac_eq_pu;
      parameter Real initvd;
      parameter Real initvq;
      Modelica.Blocks.Interfaces.RealInput Id_ref annotation (Placement(transformation(extent={{-142,40},{-102,80}})));
      Modelica.Blocks.Interfaces.RealInput Id annotation (Placement(transformation(extent={{-144,-2},{-104,38}})));
      Modelica.Blocks.Interfaces.RealInput Iqref annotation (Placement(transformation(extent={{-144,-80},{-104,-40}})));
      Modelica.Blocks.Sources.Constant const(k=Lac_eq_pu) annotation (Placement(transformation(extent={{-88,-8},{-74,6}})));
      Modelica.Blocks.Math.Product product annotation (Placement(transformation(extent={{-46,14},{-34,26}})));
      Modelica.Blocks.Math.Product product1 annotation (Placement(transformation(extent={{-40,-24},{-28,-12}})));
      PI pI(
        init_value=initvd,
        max=0.6,
        min=-0.6,
        KI=Ictrl_KI,
        KP=Ictrl_KP) annotation (Placement(transformation(extent={{-30,46},{-10,66}})));
      PI pI1(
        init_value=initvq,
        max=0.6,
        min=-0.6,
        KI=Ictrl_KI,
        KP=Ictrl_KP) annotation (Placement(transformation(extent={{-30,-56},{-10,-36}})));
      Modelica.Blocks.Math.Add add(k2=-1) annotation (Placement(transformation(extent={{-74,50},{-54,70}})));
      Modelica.Blocks.Math.Add add1(k1=-1, k2=+1) annotation (Placement(transformation(extent={{-78,-56},{-58,-36}})));
      Modelica.Blocks.Interfaces.RealInput Iq annotation (Placement(transformation(extent={{-144,-42},{-104,-2}})));
      Modelica.Blocks.Math.Add3 add3_1(k2=-1) annotation (Placement(transformation(extent={{18,46},{38,66}})));
      Modelica.Blocks.Interfaces.RealInput vd annotation (Placement(transformation(extent={{-142,70},{-102,110}})));
      Modelica.Blocks.Interfaces.RealInput vq annotation (Placement(transformation(extent={{-146,-114},{-106,-74}})));
      Modelica.Blocks.Math.Add3 add3_2(k2=-1, k1=-1) annotation (Placement(transformation(extent={{20,-56},{40,-36}})));
      Modelica.Blocks.Interfaces.RealOutput vdref annotation (Placement(transformation(extent={{100,46},{120,66}})));
      Modelica.Blocks.Interfaces.RealOutput vqref annotation (Placement(transformation(extent={{100,-56},{120,-36}})));
      Modelica.Blocks.Continuous.Filter filter(
        filterType=Modelica.Blocks.Types.FilterType.LowPass,
        analogFilter=Modelica.Blocks.Types.AnalogFilter.Bessel,
        order=1,
        f_cut=11) annotation (Placement(transformation(extent={{-40,84},{-28,96}})));
      Modelica.Blocks.Continuous.Filter filter1(
        filterType=Modelica.Blocks.Types.FilterType.LowPass,
        analogFilter=Modelica.Blocks.Types.AnalogFilter.Bessel,
        order=1,
        f_cut=11) annotation (Placement(transformation(extent={{-44,-98},{-32,-86}})));
    equation

      connect(const.y, product.u2) annotation (Line(points={{-73.3,-1},{-56,-1},{-56,16.4},{-47.2,16.4}}, color={0,0,127}));
      connect(const.y, product1.u1) annotation (Line(points={{-73.3,-1},{-56,-1},{-56,-14.4},{-41.2,-14.4}}, color={0,0,127}));
      connect(Id, add.u2) annotation (Line(points={{-124,18},{-90,18},{-90,54},{-76,54}}, color={0,0,127}));
      connect(Id_ref, add.u1) annotation (Line(points={{-122,60},{-102,60},{-76,60},{-76,66}}, color={0,0,127}));
      connect(add.y, pI.u) annotation (Line(points={{-53,60},{-40,60},{-40,56},{-32.2,56}}, color={0,0,127}));
      connect(Id, product.u1) annotation (Line(points={{-124,18},{-82,18},{-82,23.6},{-47.2,23.6}}, color={0,0,127}));
      connect(Iq, add1.u1) annotation (Line(points={{-124,-22},{-88,-22},{-88,-40},{-80,-40}}, color={0,0,127}));
      connect(Iqref, add1.u2) annotation (Line(points={{-124,-60},{-106,-60},{-86,-60},{-86,-52},{-80,-52}}, color={0,0,127}));
      connect(Iq, product1.u2) annotation (Line(points={{-124,-22},{-41.2,-22},{-41.2,-21.6}}, color={0,0,127}));
      connect(add1.y, pI1.u) annotation (Line(points={{-57,-46},{-32.2,-46}}, color={0,0,127}));
      connect(pI1.y, add3_2.u2) annotation (Line(points={{-9,-46},{18,-46}}, color={0,0,127}));
      connect(pI.y, add3_1.u2) annotation (Line(points={{-9,56},{10,56},{16,56}}, color={0,0,127}));
      connect(product1.y, add3_1.u3) annotation (Line(points={{-27.4,-18},{-12,-18},{6,-18},{6,48},{16,48}}, color={0,0,127}));
      connect(product.y, add3_2.u1) annotation (Line(points={{-33.4,20},{-6,20},{-6,-38},{18,-38}}, color={0,0,127}));
      connect(vqref, add3_2.y) annotation (Line(points={{110,-46},{41,-46}}, color={0,0,127}));
      connect(vdref, add3_1.y) annotation (Line(points={{110,56},{74,56},{39,56}}, color={0,0,127}));
      connect(vd, filter.u) annotation (Line(points={{-122,90},{-41.2,90}}, color={0,0,127}));
      connect(filter.y, add3_1.u1) annotation (Line(points={{-27.4,90},{0,90},{0,64},{16,64}}, color={0,0,127}));
      connect(vq, filter1.u) annotation (Line(points={{-126,-94},{-45.2,-94},{-45.2,-92}}, color={0,0,127}));
      connect(filter1.y, add3_2.u3) annotation (Line(points={{-31.4,-92},{-14,-92},{-14,-86},{8,-86},{8,-54},{18,-54}}, color={0,0,127}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end InnerControl;

    model DQToABC

      Modelica.Blocks.Math.RectangularToPolar rectangularToPolar annotation (Placement(transformation(extent={{-74,-10},{-54,10}})));
      Modelica.Blocks.Interfaces.RealInput vdc annotation (Placement(transformation(extent={{-142,50},{-102,90}})));
      Modelica.Blocks.Interfaces.RealInput vdref annotation (Placement(transformation(extent={{-146,0},{-106,40}})));
      Modelica.Blocks.Interfaces.RealInput vqref annotation (Placement(transformation(extent={{-148,-40},{-108,0}})));
      Modelica.Blocks.Interfaces.RealInput theta annotation (Placement(transformation(extent={{-148,-88},{-108,-48}})));
      Modelica.Blocks.Math.Division division annotation (Placement(transformation(extent={{-46,46},{-26,66}})));
      Modelica.Blocks.Nonlinear.Limiter limiter(uMax=1.5, uMin=0) annotation (Placement(transformation(extent={{-10,48},{6,64}})));
      Modelica.Blocks.Math.PolarToRectangular polarToRectangular annotation (Placement(transformation(extent={{24,46},{40,62}})));
      ClarkInv clarkInv annotation (Placement(transformation(extent={{110,-4},{130,16}})));
      Modelica.Blocks.Math.Sin sin annotation (Placement(transformation(extent={{-72,-54},{-52,-34}})));
      Modelica.Blocks.Math.Cos cos annotation (Placement(transformation(extent={{-72,-86},{-52,-66}})));
      Modelica.Blocks.Math.Product product annotation (Placement(transformation(extent={{54,42},{66,54}})));
      Modelica.Blocks.Math.Product product1 annotation (Placement(transformation(extent={{52,16},{64,28}})));
      Modelica.Blocks.Math.Product product2 annotation (Placement(transformation(extent={{42,-54},{54,-42}})));
      Modelica.Blocks.Math.Product product3 annotation (Placement(transformation(extent={{42,-78},{54,-66}})));
      Modelica.Blocks.Math.Add add(k2=+1) annotation (Placement(transformation(extent={{82,30},{96,44}})));
      Modelica.Blocks.Math.Add add1(k2=+1, k1=-1) annotation (Placement(transformation(extent={{70,-68},{84,-54}})));
      Modelica.Blocks.Sources.Constant const(k=0) annotation (Placement(transformation(extent={{60,-8},{74,6}})));
      Modelica.Blocks.Interfaces.RealOutput varef annotation (Placement(transformation(extent={{140,38},{160,58}})));
      Modelica.Blocks.Interfaces.RealOutput vbref annotation (Placement(transformation(extent={{140,-8},{160,12}})));
      Modelica.Blocks.Interfaces.RealOutput vcref annotation (Placement(transformation(extent={{140,-54},{160,-34}})));
    equation
      connect(vdref, rectangularToPolar.u_re) annotation (Line(points={{-126,20},{-84,20},{-84,6},{-76,6}}, color={0,0,127}));
      connect(vqref, rectangularToPolar.u_im) annotation (Line(points={{-128,-20},{-106,-20},{-106,-18},{-82,-18},{-82,-6},{-76,-6}}, color={0,0,127}));
      connect(rectangularToPolar.y_abs, division.u1) annotation (Line(points={{-53,6},{-42,6},{-42,28},{-74,28},{-74,62},{-48,62}}, color={0,0,127}));
      connect(vdc, division.u2) annotation (Line(points={{-122,70},{-88,70},{-88,50},{-48,50}}, color={0,0,127}));
      connect(division.y, limiter.u) annotation (Line(points={{-25,56},{-11.6,56}}, color={0,0,127}));
      connect(limiter.y, polarToRectangular.u_abs) annotation (Line(points={{6.8,56},{14,56},{14,58.8},{22.4,58.8}}, color={0,0,127}));
      connect(rectangularToPolar.y_arg, polarToRectangular.u_arg) annotation (Line(points={{-53,-6},{-20,-6},{-20,38},{14,38},{14,49.2},{22.4,49.2}}, color={0,0,127}));
      connect(theta, sin.u) annotation (Line(points={{-128,-68},{-92,-68},{-92,-44},{-74,-44}}, color={0,0,127}));
      connect(cos.u, sin.u) annotation (Line(points={{-74,-76},{-92,-76},{-92,-44},{-74,-44}}, color={0,0,127}));
      connect(polarToRectangular.y_re, product.u1) annotation (Line(points={{40.8,58.8},{48,58.8},{48,51.6},{52.8,51.6}}, color={0,0,127}));
      connect(sin.y, product.u2) annotation (Line(points={{-51,-44},{-30,-44},{28,-44},{28,44.4},{52.8,44.4}}, color={0,0,127}));
      connect(polarToRectangular.y_re, product2.u1) annotation (Line(points={{40.8,58.8},{44,58.8},{44,-36},{34,-36},{34,-44.4},{40.8,-44.4}}, color={0,0,127}));
      connect(polarToRectangular.y_im, product1.u1) annotation (Line(points={{40.8,49.2},{46,49.2},{46,25.6},{50.8,25.6}}, color={0,0,127}));
      connect(product3.u1, product1.u1) annotation (Line(points={{40.8,-68.4},{32,-68.4},{32,34},{46,34},{46,25.6},{50.8,25.6}}, color={0,0,127}));
      connect(cos.y, product1.u2) annotation (Line(points={{-51,-76},{-34,-76},{-34,-80},{12,-80},{12,18.4},{50.8,18.4}}, color={0,0,127}));
      connect(product.y, add.u1) annotation (Line(points={{66.6,48},{76,48},{76,41.2},{80.6,41.2}}, color={0,0,127}));
      connect(product1.y, add.u2) annotation (Line(points={{64.6,22},{72,22},{72,32.8},{80.6,32.8}}, color={0,0,127}));
      connect(add.y, clarkInv.A) annotation (Line(points={{96.7,37},{102,37},{102,11},{107.2,11}}, color={0,0,127}));
      connect(cos.y, product2.u2) annotation (Line(points={{-51,-76},{-16,-76},{-16,-51.6},{40.8,-51.6}}, color={0,0,127}));
      connect(sin.y, product3.u2) annotation (Line(points={{-51,-44},{-24,-44},{-24,-75.6},{40.8,-75.6}}, color={0,0,127}));
      connect(product2.y, add1.u1) annotation (Line(points={{54.6,-48},{58,-48},{68.6,-48},{68.6,-56.8}}, color={0,0,127}));
      connect(product3.y, add1.u2) annotation (Line(points={{54.6,-72},{62,-72},{62,-65.2},{68.6,-65.2}}, color={0,0,127}));
      connect(add1.y, clarkInv.B) annotation (Line(points={{84.7,-61},{86,-61},{86,5.4},{107,5.4}}, color={0,0,127}));
      connect(const.y, clarkInv.C) annotation (Line(points={{74.7,-1},{91.35,-1},{91.35,0.2},{107.2,0.2}}, color={0,0,127}));
      connect(clarkInv.a, varef) annotation (Line(points={{131,10.8},{136,10.8},{136,48},{150,48}}, color={0,0,127}));
      connect(clarkInv.b, vbref) annotation (Line(points={{131,5.8},{136,5.8},{136,2},{150,2}}, color={0,0,127}));
      connect(clarkInv.c, vcref) annotation (Line(points={{131,1.2},{136,1.2},{136,-44},{150,-44}}, color={0,0,127}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{140,100}})), Icon(coordinateSystem(extent={{-100,-100},{140,100}})));
    end DQToABC;

    model ClarkInv

      Modelica.Blocks.Interfaces.RealInput A annotation (Placement(transformation(extent={{-148,30},{-108,70}})));
      Modelica.Blocks.Interfaces.RealInput B annotation (Placement(transformation(extent={{-150,-26},{-110,14}})));
      Modelica.Blocks.Interfaces.RealInput C annotation (Placement(transformation(extent={{-148,-78},{-108,-38}})));
      Modelica.Blocks.Interfaces.RealOutput a annotation (Placement(transformation(extent={{100,38},{120,58}})));
      Modelica.Blocks.Interfaces.RealOutput b annotation (Placement(transformation(extent={{100,-12},{120,8}})));
      Modelica.Blocks.Interfaces.RealOutput c annotation (Placement(transformation(extent={{100,-58},{120,-38}})));
    equation
      a = A + C;
      b = (1/2)*(-A + sqrt(3)*B + 2*C);
      c = (1/2)*(-A - sqrt(3)*B + C);
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end ClarkInv;

    package PLL

      model Avg_value_mean_freq
        import VSC = VSCHVDC;
        parameter Real timeStep;

        Modelica.Blocks.Interfaces.RealInput inp annotation (Placement(transformation(extent={{-140,44},{-100,84}})));
        Modelica.Blocks.Interfaces.RealInput frequency annotation (Placement(transformation(extent={{-140,-102},{-100,-62}})));
        Modelica.Blocks.Interfaces.RealOutput average_output annotation (Placement(transformation(extent={{100,-10},{120,10}})));
        Modelica.Blocks.Nonlinear.Limiter limiter(uMax=1e6, uMin=1e-6) annotation (Placement(transformation(extent={{-82,68},{-62,88}})));
        Modelica.Blocks.Nonlinear.VariableDelay variableDelay(delayMax=1e100)  annotation (Placement(transformation(extent={{-6,-24},{14,-4}})));
        Modelica.Blocks.Continuous.Integrator integrator1 annotation (Placement(transformation(extent={{-68,0},{-48,20}})));
        Modelica.Blocks.Nonlinear.FixedDelay fixedDelay(delayTime=timeStep) annotation (Placement(transformation(extent={{-12,34},{2,48}})));
        Modelica.Blocks.Math.Add add annotation (Placement(transformation(extent={{68,-36},{88,-16}})));

        division division1(TimeStep=timeStep) annotation (Placement(transformation(extent={{-44,76},{-32,88}})));
        invDivision invDivision1(TimeStep=timeStep) annotation (Placement(transformation(extent={{18,78},{36,92}})));
        four_input four_input2 annotation (Placement(transformation(extent={{48,48},{68,68}})));
        Modelica.Blocks.Sources.Step step(
          height=1,
          offset=0,
          startTime=98) annotation (Placement(transformation(extent={{14,-70},{30,-54}})));
        VSC.HVDC_Emtp.PLL.three_inpt three_inpt annotation (Placement(transformation(extent={{30,-32},{50,-12}})));
        VSC.HVDC_Emtp.PLL.three_inpt2 three_inpt2_1 annotation (Placement(transformation(extent={{54,-72},{74,-52}})));
        VSC.HVDC_Emtp.PLL.truncation_block truncation annotation (Placement(transformation(extent={{-18,82},{0,98}})));
      equation
        connect(frequency, limiter.u) annotation (Line(
            points={{-120,-82},{-96,-82},{-96,78},{-84,78}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(integrator1.y, variableDelay.u) annotation (Line(
            points={{-47,10},{-40,10},{-40,-14},{-8,-14}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(inp, integrator1.u) annotation (Line(
            points={{-120,64},{-86,64},{-86,10},{-70,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(fixedDelay.u, inp) annotation (Line(
            points={{-13.4,41},{-54,41},{-54,56},{-86,56},{-86,64},{-120,64}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(limiter.y, division1.u) annotation (Line(points={{-61,78},{-54,78},{-54,82},{-45.2,82}}, color={0,0,127}));
        connect(division1.y, four_input2.u1) annotation (Line(points={{-31.76,82},{-26,82},{-26,65.8},{46.2,65.8}}, color={0,0,127}));
        connect(fixedDelay.y, four_input2.u4) annotation (Line(points={{2.7,41},{12,41},{12,49.2},{46,49.2}}, color={0,0,127}));
        connect(inp, four_input2.u3) annotation (Line(points={{-120,64},{-82,64},{-38,64},{-38,55.4},{46,55.4}}, color={0,0,127}));
        connect(invDivision1.y, variableDelay.delayTime) annotation (Line(points={{36.9,85},{78,85},{78,8},{-26,8},{-26,-20},{-8,-20}}, color={0,0,127}));
        connect(four_input2.out, add.u1) annotation (Line(points={{69,58},{88,58},{88,0},{60,0},{60,-20},{66,-20}}, color={0,0,127}));
        connect(variableDelay.y, three_inpt.u1) annotation (Line(points={{15,-14},{18,-14},{18,-13.6},{28,-13.6}}, color={0,0,127}));
        connect(three_inpt.u2, variableDelay.u) annotation (Line(points={{28,-22},{18,-22},{18,-38},{-36,-38},{-36,-14},{-8,-14}}, color={0,0,127}));
        connect(frequency, three_inpt.u3) annotation (Line(points={{-120,-82},{-16,-82},{-16,-29.6},{28,-29.6}}, color={0,0,127}));
        connect(three_inpt.out, add.u2) annotation (Line(points={{51,-22},{56,-22},{56,-32},{66,-32}}, color={0,0,127}));
        connect(step.y, three_inpt2_1.u2) annotation (Line(points={{30.8,-62},{52.2,-62}}, color={0,0,127}));
        connect(add.y, three_inpt2_1.u1) annotation (Line(points={{89,-26},{94,-26},{94,-42},{44,-42},{44,-53.6},{52,-53.6}}, color={0,0,127}));
        connect(three_inpt2_1.u3, integrator1.u) annotation (Line(points={{52,-69.6},{36,-69.6},{36,-86},{-86,-86},{-86,10},{-70,10}}, color={0,0,127}));
        connect(three_inpt2_1.out, average_output) annotation (Line(points={{75,-62},{84,-62},{84,-60},{98,-60},{98,0},{110,0}}, color={0,0,127}));
        connect(division1.y, truncation.u) annotation (Line(points={{-31.76,82},{-24,82},{-24,90},{-19.8,90}}, color={0,0,127}));
        connect(truncation.y, four_input2.u2) annotation (Line(points={{0.9,90.16},{10,90.16},{10,60.6},{46,60.6}}, color={0,0,127}));
        connect(truncation.y, invDivision1.u) annotation (Line(points={{0.9,90.16},{10,90.16},{10,85},{16.02,85}}, color={0,0,127}));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
      end Avg_value_mean_freq;

      model division
        parameter Real TimeStep;
        Modelica.Blocks.Interfaces.RealInput u annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
        Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{94,-10},{114,10}})));

      equation
        y = (1/u)/TimeStep;

      end division;

      model truncation

        Modelica.Blocks.Interfaces.RealInput trancaton annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
        Modelica.Blocks.Interfaces.RealOutput tran_out annotation (Placement(transformation(extent={{100,-10},{120,10}})));

      equation
        tran_out = integer(trancaton) + 1;

        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics));
      end truncation;

      model truncation_block

        Modelica.Blocks.Interfaces.RealInput u annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
        Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{100,-8},{120,12}})));

      equation
        y = if (u > 0) then integer(floor(u) + 1) else integer(ceil(u) + 1);

        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics));
      end truncation_block;

      model invDivision
        parameter Real TimeStep;
        Modelica.Blocks.Interfaces.RealInput u annotation (Placement(transformation(extent={{-142,-20},{-102,20}})));
        Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      equation
        y = TimeStep*u;
      end invDivision;

      model four_input

        Modelica.Blocks.Interfaces.RealInput u1 annotation (Placement(transformation(extent={{-138,58},{-98,98}})));
        Modelica.Blocks.Interfaces.RealInput u2 annotation (Placement(transformation(extent={{-140,6},{-100,46}})));
        Modelica.Blocks.Interfaces.RealInput u3 annotation (Placement(transformation(extent={{-140,-46},{-100,-6}})));
        Modelica.Blocks.Interfaces.RealInput u4 annotation (Placement(transformation(extent={{-140,-108},{-100,-68}})));
        Modelica.Blocks.Interfaces.RealOutput out annotation (Placement(transformation(extent={{100,-10},{120,10}})));

      equation
        out = (u1 - u2)/u1*(u3 + ((u3 - u4)*(u1 - u2))/2);

        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics));
      end four_input;

      model three_inpt

        Modelica.Blocks.Interfaces.RealInput u2 annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
        Modelica.Blocks.Interfaces.RealInput u3 annotation (Placement(transformation(extent={{-140,-96},{-100,-56}})));
        Modelica.Blocks.Interfaces.RealOutput out annotation (Placement(transformation(extent={{100,-10},{120,10}})));
        Modelica.Blocks.Interfaces.RealInput u1 annotation (Placement(transformation(extent={{-140,64},{-100,104}})));

      equation
        out = (u2 - u1)*u3;

        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics));
      end three_inpt;

      model three_inpt2

        Modelica.Blocks.Interfaces.RealInput u2 annotation (Placement(transformation(extent={{-138,-20},{-98,20}})));
        Modelica.Blocks.Interfaces.RealInput u3 annotation (Placement(transformation(extent={{-140,-96},{-100,-56}})));
        Modelica.Blocks.Interfaces.RealOutput out annotation (Placement(transformation(extent={{100,-10},{120,10}})));
        Modelica.Blocks.Interfaces.RealInput u1 annotation (Placement(transformation(extent={{-140,64},{-100,104}})));

      equation
        if u2 >= 0 then
          out = u1;
        else
          out = u2;
        end if;

        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
      end three_inpt2;

      model modulo_operation " block to perform modulo_operation fro PLL"

        constant Real pi=Modelica.Constants.pi;
        Real p;

        Modelica.Blocks.Interfaces.RealInput u annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
        Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{100,-8},{120,12}})));

      equation
        p = 2*pi;
        y = mod(u, p);

      end modulo_operation;

      model PLL_Full
        parameter Real Freq;
        parameter Real time_step;
        import Modelica.Constants.pi;
        parameter Real init=(Freq*pi*2)/1600;
        parameter Real Kp=80;
        parameter Real Ki=1600;
        parameter Real Umin=0.75*2*pi*Freq;
        parameter Real Umax=1.21*2*pi*Freq;
        parameter Real C=2*pi;
        park_V park_V1 annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
        Modelica.Blocks.Interfaces.RealInput va annotation (Placement(transformation(extent={{-172,52},{-132,92}})));
        Modelica.Blocks.Interfaces.RealInput vb annotation (Placement(transformation(extent={{-166,-14},{-126,26}})));
        Avg_value_mean_freq avg_value_mean_freq(timeStep=time_step) annotation (Placement(transformation(extent={{-40,32},{-22,48}})));
        Modelica.Blocks.Continuous.Integrator integrator(initType=Modelica.Blocks.Types.Init.InitialOutput, y_start=init) annotation (Placement(transformation(extent={{-4,38},{10,52}})));
        Modelica.Blocks.Math.Add add(k1=Ki, k2=Kp) annotation (Placement(transformation(extent={{22,38},{38,54}})));
        Modelica.Blocks.Nonlinear.Limiter limiter(uMax=Umax, uMin=Umin) annotation (Placement(transformation(extent={{16,72},{30,86}})));
        Modelica.Blocks.Continuous.Integrator integrator1(initType=Modelica.Blocks.Types.Init.NoInit) annotation (Placement(transformation(extent={{46,72},{60,86}})));
        modulo_operation modulo_operation1 annotation (Placement(transformation(extent={{74,72},{88,86}})));
        Modelica.Blocks.Interfaces.RealOutput ThetaRad annotation (Placement(transformation(extent={{120,50},{140,70}})));
        Modelica.Blocks.Math.Division division annotation (Placement(transformation(extent={{28,-10},{42,4}})));
        Modelica.Blocks.Sources.Constant const(k=C) annotation (Placement(transformation(extent={{-20,-12},{-8,0}})));
        Limiter_variation limiter_variation(timeStep=time_step) annotation (Placement(transformation(extent={{54,-2},{70,16}})));
        Modelica.Blocks.Nonlinear.FixedDelay fixedDelay(delayTime=time_step) annotation (Placement(transformation(extent={{92,-28},{106,-14}})));
        Modelica.Blocks.Interfaces.RealOutput FreqHz(start=50) annotation (Placement(transformation(extent={{126,-32},{146,-12}})));
        Modelica.Blocks.Continuous.Filter filter(
          filterType=Modelica.Blocks.Types.FilterType.LowPass,
          f_cut=30,
          analogFilter=Modelica.Blocks.Types.AnalogFilter.Bessel) annotation (Placement(transformation(extent={{84,4},{104,24}})));
      equation
        connect(va, park_V1.Valpha) annotation (Line(points={{-152,72},{-96,72},{-96,38},{-82,38}}, color={0,0,127}));
        connect(vb, park_V1.Vbeta) annotation (Line(points={{-146,6},{-126,6},{-126,8},{-106,8},{-106,30},{-82,30}}, color={0,0,127}));
        connect(park_V1.Vq, avg_value_mean_freq.inp) annotation (Line(points={{-59,30},{-48,30},{-48,45.12},{-41.8,45.12}}, color={0,0,127}));
        connect(avg_value_mean_freq.average_output, integrator.u) annotation (Line(points={{-21.1,40},{-14,40},{-14,45},{-5.4,45}},color={0,0,127}));
        connect(avg_value_mean_freq.average_output, add.u2) annotation (Line(points={{-21.1,40},{-16,40},{-16,24},{20,24},{20,41.2},{20.4,41.2}}, color={0,0,127}));
        connect(integrator.y, add.u1) annotation (Line(points={{10.7,45},{16,45},{16,50.8},{20.4,50.8}}, color={0,0,127}));
        connect(add.y, limiter.u) annotation (Line(points={{38.8,46},{48,46},{48,62},{10,62},{10,79},{14.6,79}}, color={0,0,127}));
        connect(limiter.y, integrator1.u) annotation (Line(points={{30.7,79},{37.35,79},{44.6,79}}, color={0,0,127}));
        connect(integrator1.y, modulo_operation1.u) annotation (Line(points={{60.7,79},{68.35,79},{72.6,79}}, color={0,0,127}));
        connect(modulo_operation1.y, ThetaRad) annotation (Line(points={{88.7,79.14},{108,79.14},{108,60},{130,60}}, color={0,0,127}));
        connect(ThetaRad, park_V1.theta) annotation (Line(points={{130,60},{150,60},{150,62},{150,96},{150,98},{-92,98},{-92,22.2},{-83,22.2}}, color={0,0,127}));
        connect(const.y, division.u2) annotation (Line(points={{-7.4,-6},{6,-6},{6,-7.2},{26.6,-7.2}}, color={0,0,127}));
        connect(limiter.y, division.u1) annotation (Line(points={{30.7,79},{40,79},{40,12},{16,12},{16,1.2},{26.6,1.2}}, color={0,0,127}));
        connect(division.y, limiter_variation.inp) annotation (Line(points={{42.7,-3},{48,-3},{48,7},{52.4,7}}, color={0,0,127}));
        connect(fixedDelay.y, FreqHz) annotation (Line(points={{106.7,-21},{117.35,-21},{117.35,-22},{136,-22}}, color={0,0,127}));
        connect(avg_value_mean_freq.frequency, FreqHz)
          annotation (Line(points={{-41.8,33.44},{-41.8,-26},{68,-26},{68,-48},{112,-48},{112,-22},{114,-22},{117.35,-21},{117.35,-22},{136,-22}}, color={0,0,127}));
        connect(limiter_variation.out, filter.u) annotation (Line(points={{70.8,7},{78,7},{78,14},{82,14}}, color={0,0,127}));
        connect(filter.y, fixedDelay.u) annotation (Line(points={{105,14},{118,14},{118,-6},{86,-6},{86,-21},{90.6,-21}}, color={0,0,127}));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{120,100}})), Icon(coordinateSystem(extent={{-120,-100},{120,100}})));
      end PLL_Full;

      model park_V " Park block of PLL"

        Modelica.Blocks.Interfaces.RealInput Valpha annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
        Modelica.Blocks.Interfaces.RealInput Vbeta annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
        Modelica.Blocks.Interfaces.RealOutput Vd annotation (Placement(transformation(extent={{100,68},{120,88}})));
        Modelica.Blocks.Interfaces.RealOutput Vq annotation (Placement(transformation(extent={{100,-10},{120,10}})));
        Modelica.Blocks.Interfaces.RealOutput zero_sequence annotation (Placement(transformation(extent={{100,-90},{120,-70}})));
        Modelica.Blocks.Interfaces.RealInput theta annotation (Placement(transformation(
              extent={{20,-20},{-20,20}},
              rotation=180,
              origin={-130,-78})));
        Modelica.Blocks.Sources.Constant const(k=0) annotation (Placement(transformation(extent={{-66,-52},{-46,-32}})));
      equation

        Vd = Valpha*sin(theta) - Vbeta*cos(theta);
        Vq = Valpha*cos(theta) + Vbeta*sin(theta);

        connect(const.y, zero_sequence) annotation (Line(points={{-45,-42},{22,-42},{22,-80},{110,-80}}, color={0,0,127}));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
      end park_V;

      model Limiter_variation "Limiter_Variation block of PLL"
        parameter Real timeStep;
        Modelica.Blocks.Interfaces.RealInput inp annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
        Modelica.Blocks.Math.Add add(k2=-1) annotation (Placement(transformation(extent={{-68,-18},{-48,2}})));
        Modelica.Blocks.Nonlinear.Limiter limiter(uMax=timeStep*12, uMin=-timeStep*12) annotation (Placement(transformation(extent={{-30,-14},{-10,6}})));
        Modelica.Blocks.Math.Add add1 annotation (Placement(transformation(extent={{28,-20},{48,0}})));
        Modelica.Blocks.Nonlinear.FixedDelay fixedDelay(delayTime=timeStep) annotation (Placement(transformation(extent={{72,-38},{84,-26}})));
        Modelica.Blocks.Interfaces.RealOutput out annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      equation
        connect(inp, add.u1) annotation (Line(
            points={{-120,0},{-98,0},{-98,-2},{-70,-2}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(add.y, limiter.u) annotation (Line(
            points={{-47,-8},{-42,-8},{-42,-4},{-32,-4}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(add1.y, out) annotation (Line(
            points={{49,-10},{76,-10},{76,0},{110,0}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(limiter.y, add1.u1) annotation (Line(
            points={{-9,-4},{26,-4}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(add1.y, fixedDelay.u) annotation (Line(points={{49,-10},{58,-10},{58,-32},{70.8,-32}}, color={0,0,127}));
        connect(fixedDelay.y, add1.u2) annotation (Line(points={{84.6,-32},{90,-32},{90,-50},{18,-50},{18,-16},{26,-16}}, color={0,0,127}));
        connect(add.u2, add1.u2) annotation (Line(points={{-70,-14},{-86,-14},{-86,-50},{18,-50},{18,-16},{26,-16}}, color={0,0,127}));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
      end Limiter_variation;

      model PLL_Full2
        parameter Real Freq;
        parameter Real time_step;
        import Modelica.Constants.pi;
        //parameter Real init=(Freq*pi*2)/1600;
        parameter Real init=0.196350;
        parameter Real Kp=80;
        parameter Real Ki=1600;
        parameter Real Umin=0.75*2*pi*Freq;
        parameter Real Umax=1.25*2*pi*Freq;
        parameter Real C=2*pi;
        park_V park_V1 annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
        Modelica.Blocks.Interfaces.RealInput va annotation (Placement(transformation(extent={{-172,52},{-132,92}})));
        Modelica.Blocks.Interfaces.RealInput vb annotation (Placement(transformation(extent={{-166,-14},{-126,26}})));
        Modelica.Blocks.Continuous.Integrator integrator(initType=Modelica.Blocks.Types.Init.InitialOutput, y_start=init) annotation (Placement(transformation(extent={{-4,38},{10,52}})));
        Modelica.Blocks.Math.Add add(k1=Ki, k2=Kp) annotation (Placement(transformation(extent={{22,38},{38,54}})));
        Modelica.Blocks.Nonlinear.Limiter limiter(uMax=Umax, uMin=Umin) annotation (Placement(transformation(extent={{16,72},{30,86}})));
        Modelica.Blocks.Continuous.Integrator integrator1(initType=Modelica.Blocks.Types.Init.NoInit) annotation (Placement(transformation(extent={{46,72},{60,86}})));
        modulo_operation modulo_operation1 annotation (Placement(transformation(extent={{74,72},{88,86}})));
        Modelica.Blocks.Interfaces.RealOutput ThetaRad annotation (Placement(transformation(extent={{120,50},{140,70}})));
        Modelica.Blocks.Math.Division division annotation (Placement(transformation(extent={{28,-10},{42,4}})));
        Modelica.Blocks.Sources.Constant const(k=C) annotation (Placement(transformation(extent={{-20,-12},{-8,0}})));
        Limiter_variation limiter_variation(timeStep=time_step) annotation (Placement(transformation(extent={{54,-2},{70,16}})));
        Modelica.Blocks.Nonlinear.FixedDelay fixedDelay(delayTime=time_step) annotation (Placement(transformation(extent={{92,-28},{106,-14}})));
        Modelica.Blocks.Interfaces.RealOutput FreqHz(start=50) annotation (Placement(transformation(extent={{126,-32},{146,-12}})));
        VSCHVDC.HVDC_Emtp.PLL.Avg_value_mean_freq avg_value_mean_freqWithoutTrunc(timeStep=
              time_step)
          annotation (Placement(transformation(extent={{-48,10},{-28,30}})));
      equation
        connect(va, park_V1.Valpha) annotation (Line(points={{-152,72},{-96,72},{-96,38},{-82,38}}, color={0,0,127}));
        connect(vb, park_V1.Vbeta) annotation (Line(points={{-146,6},{-126,6},{-126,8},{-106,8},{-106,30},{-82,30}}, color={0,0,127}));
        connect(integrator.y, add.u1) annotation (Line(points={{10.7,45},{16,45},{16,50.8},{20.4,50.8}}, color={0,0,127}));
        connect(add.y, limiter.u) annotation (Line(points={{38.8,46},{48,46},{48,62},{10,62},{10,79},{14.6,79}}, color={0,0,127}));
        connect(limiter.y, integrator1.u) annotation (Line(points={{30.7,79},{37.35,79},{44.6,79}}, color={0,0,127}));
        connect(integrator1.y, modulo_operation1.u) annotation (Line(points={{60.7,79},{68.35,79},{72.6,79}}, color={0,0,127}));
        connect(modulo_operation1.y, ThetaRad) annotation (Line(points={{88.7,79.14},{108,79.14},{108,60},{130,60}}, color={0,0,127}));
        connect(ThetaRad, park_V1.theta) annotation (Line(points={{130,60},{150,60},{150,62},{150,96},{150,98},{-92,98},{-92,22.2},{-83,22.2}}, color={0,0,127}));
        connect(const.y, division.u2) annotation (Line(points={{-7.4,-6},{6,-6},{6,-7.2},{26.6,-7.2}}, color={0,0,127}));
        connect(limiter.y, division.u1) annotation (Line(points={{30.7,79},{40,79},{40,12},{16,12},{16,1.2},{26.6,1.2}}, color={0,0,127}));
        connect(division.y, limiter_variation.inp) annotation (Line(points={{42.7,-3},{48,-3},{48,7},{52.4,7}}, color={0,0,127}));
        connect(fixedDelay.y, FreqHz) annotation (Line(points={{106.7,-21},{117.35,-21},{117.35,-22},{136,-22}}, color={0,0,127}));
        connect(avg_value_mean_freqWithoutTrunc.average_output, integrator.u) annotation (Line(points={{-27,20},{-20,20},{-20,45},{-5.4,45}}, color={0,0,127}));
        connect(avg_value_mean_freqWithoutTrunc.average_output, add.u2) annotation (Line(points={{-27,20},{-4,20},{-4,18},{18,18},{18,41.2},{20.4,41.2}}, color={0,0,127}));
        connect(park_V1.Vq, avg_value_mean_freqWithoutTrunc.inp) annotation (Line(points={{-59,30},{-54,30},{-54,26.4},{-50,26.4}}, color={0,0,127}));
        connect(fixedDelay.y, avg_value_mean_freqWithoutTrunc.frequency) annotation (Line(points={{106.7,-21},{118,-21},{118,-44},{-68,-44},{-68,11.8},{-50,11.8}}, color={0,0,127}));
        connect(limiter_variation.out, fixedDelay.u) annotation (Line(points={{70.8,7},{76,7},{76,-21},{90.6,-21}}, color={0,0,127}));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{120,100}})), Icon(coordinateSystem(extent={{-120,-100},{120,100}})));
      end PLL_Full2;

      model Avg_value_mean_freqWithoutTrunc
        import VSC = VSCHVDC;
        parameter Real timeStep;

        Modelica.Blocks.Interfaces.RealInput inp annotation (Placement(transformation(extent={{-140,44},{-100,84}})));
        Modelica.Blocks.Interfaces.RealInput frequency annotation (Placement(transformation(extent={{-140,-102},{-100,-62}})));
        Modelica.Blocks.Interfaces.RealOutput average_output annotation (Placement(transformation(extent={{100,-10},{120,10}})));
        Modelica.Blocks.Nonlinear.Limiter limiter(uMax=1e6, uMin=1e-6) annotation (Placement(transformation(extent={{-82,68},{-62,88}})));
        Modelica.Blocks.Nonlinear.VariableDelay variableDelay(delayMax=0.0233) annotation (Placement(transformation(extent={{-6,-24},{14,-4}})));
        Modelica.Blocks.Continuous.Integrator integrator1 annotation (Placement(transformation(extent={{-68,0},{-48,20}})));
        Modelica.Blocks.Nonlinear.FixedDelay fixedDelay(delayTime=timeStep) annotation (Placement(transformation(extent={{-12,34},{2,48}})));
        Modelica.Blocks.Math.Add add annotation (Placement(transformation(extent={{68,-36},{88,-16}})));

        division division1(TimeStep=timeStep) annotation (Placement(transformation(extent={{-44,76},{-32,88}})));
        invDivision invDivision1(TimeStep=timeStep) annotation (Placement(transformation(extent={{18,78},{36,92}})));
        four_input four_input2 annotation (Placement(transformation(extent={{48,48},{68,68}})));
        Modelica.Blocks.Sources.Step step(
          startTime=98,
          height=-1,
          offset=1) annotation (Placement(transformation(extent={{14,-70},{30,-54}})));
        VSC.HVDC_Emtp.PLL.three_inpt three_inpt annotation (Placement(transformation(extent={{30,-32},{50,-12}})));
        VSC.HVDC_Emtp.PLL.three_inpt2 three_inpt2_1 annotation (Placement(transformation(extent={{54,-72},{74,-52}})));
      equation
        connect(frequency, limiter.u) annotation (Line(
            points={{-120,-82},{-96,-82},{-96,78},{-84,78}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(integrator1.y, variableDelay.u) annotation (Line(
            points={{-47,10},{-40,10},{-40,-14},{-8,-14}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(inp, integrator1.u) annotation (Line(
            points={{-120,64},{-86,64},{-86,10},{-70,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(fixedDelay.u, inp) annotation (Line(
            points={{-13.4,41},{-54,41},{-54,56},{-86,56},{-86,64},{-120,64}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(limiter.y, division1.u) annotation (Line(points={{-61,78},{-54,78},{-54,82},{-45.2,82}}, color={0,0,127}));
        connect(division1.y, four_input2.u1) annotation (Line(points={{-31.76,82},{-26,82},{-26,65.8},{46.2,65.8}}, color={0,0,127}));
        connect(fixedDelay.y, four_input2.u4) annotation (Line(points={{2.7,41},{12,41},{12,49.2},{46,49.2}}, color={0,0,127}));
        connect(inp, four_input2.u3) annotation (Line(points={{-120,64},{-82,64},{-38,64},{-38,55.4},{46,55.4}}, color={0,0,127}));
        connect(invDivision1.y, variableDelay.delayTime) annotation (Line(points={{36.9,85},{78,85},{78,8},{-26,8},{-26,-20},{-8,-20}}, color={0,0,127}));
        connect(four_input2.out, add.u1) annotation (Line(points={{69,58},{88,58},{88,0},{60,0},{60,-20},{66,-20}}, color={0,0,127}));
        connect(variableDelay.y, three_inpt.u1) annotation (Line(points={{15,-14},{18,-14},{18,-13.6},{28,-13.6}}, color={0,0,127}));
        connect(three_inpt.u2, variableDelay.u) annotation (Line(points={{28,-22},{18,-22},{18,-38},{-36,-38},{-36,-14},{-8,-14}}, color={0,0,127}));
        connect(frequency, three_inpt.u3) annotation (Line(points={{-120,-82},{-16,-82},{-16,-29.6},{28,-29.6}}, color={0,0,127}));
        connect(three_inpt.out, add.u2) annotation (Line(points={{51,-22},{56,-22},{56,-32},{66,-32}}, color={0,0,127}));
        connect(step.y, three_inpt2_1.u2) annotation (Line(points={{30.8,-62},{52.2,-62}}, color={0,0,127}));
        connect(add.y, three_inpt2_1.u1) annotation (Line(points={{89,-26},{94,-26},{94,-42},{44,-42},{44,-53.6},{52,-53.6}}, color={0,0,127}));
        connect(three_inpt2_1.u3, integrator1.u) annotation (Line(points={{52,-69.6},{36,-69.6},{36,-86},{-86,-86},{-86,10},{-70,10}}, color={0,0,127}));
        connect(three_inpt2_1.out, average_output) annotation (Line(points={{75,-62},{84,-62},{84,-60},{98,-60},{98,0},{110,0}}, color={0,0,127}));
        connect(division1.y, invDivision1.u) annotation (Line(points={{-31.76,82},{-8,82},{-8,85},{16.02,85}}, color={0,0,127}));
        connect(division1.y, four_input2.u2) annotation (Line(points={{-31.76,82},{-18,82},{-18,80},{-2,80},{-2,60.6},{46,60.6}}, color={0,0,127}));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
      end Avg_value_mean_freqWithoutTrunc;
    end PLL;

    model ChoiceController
      parameter Integer Mode=1 annotation (Dialog(group="ControllerChoice"), choices(choice=1
            "Active Power Control",                                                                                   choice=0
            "DC Voltage Control"));
      Modelica.Blocks.Interfaces.RealInput P annotation (Placement(transformation(extent={{-146,20},{-106,60}})));
      Modelica.Blocks.Interfaces.RealInput VDC annotation (Placement(transformation(extent={{-148,-60},{-108,-20}})));
      Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    equation
      if Mode > 0 then
        y = P;
      else
        y = VDC;
      end if;
    end ChoiceController;

    model ChoiceQVAC_Controller
      parameter Integer C_QVAC=1 annotation (Dialog(group="ControllerChoice"), choices(choice=1
            "Reactive Power Control",                                                                                     choice=0
            "VAC Control"));
      Modelica.Blocks.Interfaces.RealInput Q annotation (Placement(transformation(extent={{-146,20},{-106,60}})));
      Modelica.Blocks.Interfaces.RealInput VAC annotation (Placement(transformation(extent={{-148,-60},{-108,-20}})));
      Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    equation
      if C_QVAC > 0 then
        y = Q;
      else
        y = VAC;
      end if;
    end ChoiceQVAC_Controller;

    model UpperLevel
      parameter Real VDC_ref "VDC reference value";
      parameter Real KIVDC "Integral gain of VDC controller";
      parameter Real KPVDC "Proportional gain of VDC controller";
      parameter Real init_value_OVdc
        "Initial Output of Integrator of VDC controller";
      parameter Real Vdc_step "Step (Height) value of VDC in the reference";
      parameter Real t_VdcStep "Start Time of the step";
      parameter Real KIP "Integral Gain of Active Power Regulator";
      parameter Real KPP "Proportional gain of Active Power Regulator";
      parameter Real P_ref "Active power reference (p.u.)";
      parameter Real P_step
        "Step (Height) value of Active Power, in the reference";
      parameter Real start_time_P "Start Time of the step";
      parameter Real init_P "Initial Output of Integrator of P controller";
      parameter Real P_Vdc_Kp "Gain of Deadband control";
      parameter Real KIQ "Integral gain of Q controller";
      parameter Real KPQ "Proportional gain of P controller";
      parameter Real Q_ref "Reference value of Reactive Power (p.u.)";
      parameter Real Q_step
        "Step (Height) value of Reactive Power, in the reference";
      parameter Real Qinit_value "Initial Output of Integrator of Q controller";
      parameter Real start_time_Q "Start Time of the step";
      parameter Real Vac_ref "Reference AC voltage (p.u.)";
      parameter Real Vac_step "Step (Height) value of VAC, in the reference";
      parameter Real Vac_init "Initial Output of Integrator of VAC controller";
      parameter Real Vac_step_time "Start Time of the step";
      parameter Real KIVAC "Integral Gain of VAC controller";
      parameter Real KPVAC "Proportional Gain of VAC contoller";
      parameter Integer Mode annotation (Dialog(group="ControllerChoice"), choices(choice=1
            "Active Power Control",                                                                                 choice=0
            "DC Voltage Control"));
      parameter Real Ictrl_KP;
      parameter Real Ictrl_KI;
      parameter Real Lac_eq_pu;
      parameter Real initvd;
      parameter Real initvq;
      parameter Integer C_QVAC annotation (Dialog(group="ControllerChoice"), choices(choice=1
            "Reactive Power Control",                                                                                   choice=0
            "VAC Control"));
      parameter Real Freq;
      parameter Real time_step;
      import Modelica.Constants.pi;
      parameter Real initPLL;
      parameter Real KpPLL;
      parameter Real KiPLL;
      parameter Real I_lim=1.1;
      parameter Real Id_lim=1.1;
      parameter Real Iq_lim=0.5;
      parameter Integer Prio annotation (Dialog(group="Idqlimiter"), choices(choice=1
            "Active Power",                                                                           choice=0
            "Reactive Power"));

      VDC_Controller vDC_Controller(
        VDC_ref=VDC_ref,
        KIVDC=KIVDC,
        KPVDC=KPVDC,
        init_value_OVdc=init_value_OVdc,
        Vdc_step=Vdc_step,
        t_VdcStep=t_VdcStep) annotation (Placement(transformation(extent={{-64,52},{-50,68}})));
      P_Controller p_Controller(
        KIP=KIP,
        KPP=KPP,
        P_ref=P_ref,
        P_step=P_step,
        start_time_P=start_time_P,
        init_P=init_P,
        P_Vdc_Kp=P_Vdc_Kp) annotation (Placement(transformation(extent={{-64,30},{-48,42}})));
      Q_Controller q_Controller(
        KIQ=KIQ,
        KPQ=KPQ,
        Q_ref=Q_ref,
        Q_step=Q_step,
        Qinit_value=Qinit_value,
        start_time_Q=start_time_Q) annotation (Placement(transformation(extent={{-66,10},{-50,22}})));
      VAC_Controller vAC_Controller(
        Vac_ref=Vac_ref,
        Vac_step=Vac_step,
        Vac_init=Vac_init,
        Vac_step_time=Vac_step_time,
        KIVAC=KIVAC,
        KPVAC=KPVAC) annotation (Placement(transformation(extent={{-64,-12},{-50,2}})));
      Power_Current power_Current annotation (Placement(transformation(extent={{-22,42},{-8,58}})));
      ChoiceController choiceController(Mode=Mode) annotation (Placement(transformation(extent={{12,38},{24,54}})));
      InnerControl innerControl(
        Ictrl_KP=Ictrl_KP,
        Ictrl_KI=Ictrl_KI,
        Lac_eq_pu=Lac_eq_pu,
        initvd=initvd,
        initvq=initvq) annotation (Placement(transformation(extent={{54,-12},{90,36}})));
      DQToABC dQToABC annotation (Placement(transformation(extent={{108,8},{132,28}})));
      Clark clark annotation (Placement(transformation(extent={{-126,2},{-106,22}})));
      dq_trnsfrmns.dq_transfo dq_transfo annotation (Placement(transformation(extent={{-94,-46},{-74,-26}})));
      VSCHVDC.HVDC_Emtp.PLL.PLL_Full2 pLL_Full2_1(
        Freq=Freq,
        time_step=time_step,
        init=initPLL,
        Kp=KpPLL,
        Ki=KiPLL)
        annotation (Placement(transformation(extent={{-46,-72},{-22,-52}})));
      VSCHVDC.HVDC_Emtp.SignalCalculationNoFilter signalCalculation
        annotation (Placement(transformation(extent={{-92,6},{-78,22}})));
      ChoiceQVAC_Controller choiceQVAC_Controller(C_QVAC=C_QVAC) annotation (Placement(transformation(extent={{10,20},{24,32}})));
      Modelica.Blocks.Interfaces.RealInput VDC annotation (Placement(transformation(extent={{-186,54},{-146,94}})));
      Modelica.Blocks.Interfaces.RealInput V_abc_Y[3] annotation (Placement(transformation(extent={{-186,16},{-146,56}})));
      Modelica.Blocks.Interfaces.RealInput I_abc_Y[3] annotation (Placement(transformation(extent={{-184,-24},{-144,16}})));
      Modelica.Blocks.Interfaces.RealInput V_abc_D[3] annotation (Placement(transformation(extent={{-188,-60},{-148,-20}})));
      Modelica.Blocks.Interfaces.RealInput I_abc_D[3] annotation (Placement(transformation(extent={{-188,-100},{-148,-60}})));
      Modelica.Blocks.Interfaces.RealOutput Varef annotation (Placement(transformation(extent={{142,38},{162,58}})));
      Modelica.Blocks.Interfaces.RealOutput Vbref annotation (Placement(transformation(extent={{142,-4},{162,16}})));
      Modelica.Blocks.Interfaces.RealOutput Vcref annotation (Placement(transformation(extent={{142,-40},{162,-20}})));
      IdqRef_Limiter idqRef_Limiter(
        I_lim=I_lim,
        Id_lim=Id_lim,
        Iq_lim=Iq_lim,
        Prio=Prio) annotation (Placement(transformation(extent={{42,48},{62,68}})));
      Modelica.Blocks.Continuous.Filter filter(
        filterType=Modelica.Blocks.Types.FilterType.LowPass,
        analogFilter=Modelica.Blocks.Types.AnalogFilter.Bessel,
        f_cut=16,
        order=2) annotation (Placement(transformation(extent={{-42,20},{-30,32}})));
    equation
      connect(clark.V_ab_Y[1], signalCalculation.V_al_Y) annotation (Line(points={{-105,19.5},{-99.5,19.92},{-93.54,19.92}}, color={0,0,127}));
      connect(clark.V_ab_Y[2], signalCalculation.V_be_Y) annotation (Line(points={{-105,20.5},{-100,20.5},{-100,15.92},{-93.68,15.92}}, color={0,0,127}));
      connect(clark.I_ab_Y[1], signalCalculation.I_al_Y) annotation (Line(points={{-105,14.5},{-100,14.5},{-100,12.24},{-93.82,12.24}}, color={0,0,127}));
      connect(clark.I_ab_Y[2], signalCalculation.I_be_Y) annotation (Line(points={{-105,15.5},{-100,15.5},{-100,8.56},{-93.68,8.56}}, color={0,0,127}));
      connect(clark.V_ab_Y[1], dq_transfo.V_aplha_Y) annotation (Line(points={{-105,19.5},{-100,19.5},{-100,-26},{-100,-27},{-96,-27}}, color={0,0,127}));
      connect(clark.V_ab_Y[2], dq_transfo.V_beta_Y) annotation (Line(points={{-105,20.5},{-102,20.5},{-102,-33},{-96,-33}}, color={0,0,127}));
      connect(clark.I_ab_D[1], dq_transfo.I_alpha_D) annotation (Line(points={{-105,4.1},{-104,4.1},{-104,-38.6},{-96,-38.6}}, color={0,0,127}));
      connect(clark.I_ab_D[2], dq_transfo.I_beta_D) annotation (Line(points={{-105,5.1},{-94,5.1},{-94,-16},{-110,-16},{-110,-44},{-96,-44}}, color={0,0,127}));
      connect(pLL_Full2_1.ThetaRad, dq_transfo.theta) annotation (Line(points={{-21,-56},{-12,-56},{-12,-16},{-84,-16},{-84,-24}}, color={0,0,127}));
      connect(dq_transfo.V_aplha_Y, pLL_Full2_1.va) annotation (Line(points={{-96,-27},{-122,-27},{-122,-54.8},{-49.2,-54.8}}, color={0,0,127}));
      connect(dq_transfo.V_beta_Y, pLL_Full2_1.vb) annotation (Line(points={{-96,-33},{-134,-33},{-134,-61.4},{-48.6,-61.4}}, color={0,0,127}));
      connect(signalCalculation.P, p_Controller.P) annotation (Line(points={{-77.3,18.48},{-72,18.48},{-72,34.44},{-66.08,34.44}}, color={0,0,127}));
      connect(signalCalculation.Q, q_Controller.Q) annotation (Line(points={{-77.3,14},{-72,14},{-72,15.28},{-67.76,15.28}}, color={0,0,127}));
      connect(signalCalculation.V_ac_Grid, vAC_Controller.VAC) annotation (Line(points={{-77.3,10.32},{-70,10.32},{-70,-5.84},{-65.4,-5.84}}, color={0,0,127}));
      connect(p_Controller.PREF, power_Current.P_REF) annotation (Line(points={{-47.2,36},{-42,36},{-42,55.6},{-24.1,55.6}}, color={0,0,127}));
      connect(q_Controller.QREF, power_Current.Q_REF) annotation (Line(points={{-49.2,16},{-38,16},{-38,50.96},{-24.1,50.96}}, color={0,0,127}));
      connect(power_Current.Id_Ref, choiceController.P) annotation (Line(points={{-7.3,52.88},{2,52.88},{2,49.2},{10.44,49.2}}, color={0,0,127}));
      connect(vDC_Controller.y, choiceController.VDC) annotation (Line(points={{-49.3,60},{-24,60},{-2,60},{-2,42.8},{10.32,42.8}}, color={0,0,127}));
      connect(dq_transfo.Iq, vAC_Controller.Iq) annotation (Line(points={{-73,-44},{-62,-44},{-62,-20},{-74,-20},{-74,-9.2},{-65.4,-9.2}}, color={0,0,127}));
      connect(power_Current.Iq_Ref, choiceQVAC_Controller.Q) annotation (Line(points={{-7.3,47.76},{2,47.76},{2,28.4},{8.18,28.4}}, color={0,0,127}));
      connect(vAC_Controller.Idref, choiceQVAC_Controller.VAC) annotation (Line(points={{-49.16,-5},{-20,-5},{-20,23.6},{8.04,23.6}}, color={0,0,127}));
      connect(dq_transfo.Vd, innerControl.vd) annotation (Line(points={{-73,-27},{40,-27},{40,33.6},{50.04,33.6}}, color={0,0,127}));
      connect(dq_transfo.Id, innerControl.Id) annotation (Line(points={{-73,-37.8},{30,-37.8},{30,16.32},{49.68,16.32}}, color={0,0,127}));
      connect(dq_transfo.Iq, innerControl.Iq) annotation (Line(points={{-73,-44},{-24,-44},{20,-44},{20,6.72},{49.68,6.72}}, color={0,0,127}));
      connect(dq_transfo.Vq, innerControl.vq) annotation (Line(points={{-73,-32},{-14,-32},{-14,-10.56},{49.32,-10.56}}, color={0,0,127}));
      connect(innerControl.vdref, dQToABC.vdref) annotation (Line(points={{91.8,25.44},{98,25.44},{98,20},{105.4,20}}, color={0,0,127}));
      connect(innerControl.vqref, dQToABC.vqref) annotation (Line(points={{91.8,0.96},{96,0.96},{96,16},{105.2,16}}, color={0,0,127}));
      connect(pLL_Full2_1.ThetaRad, dQToABC.theta) annotation (Line(points={{-21,-56},{100,-56},{100,11.2},{105.2,11.2}}, color={0,0,127}));
      connect(V_abc_Y[1], clark.V_abc_Y[1]);
      connect(V_abc_Y[2], clark.V_abc_Y[2]);
      connect(V_abc_Y[3], clark.V_abc_Y[3]);

      connect(I_abc_Y[1], clark.I_abc_Y[1]);
      connect(I_abc_Y[2], clark.I_abc_Y[2]);
      connect(I_abc_Y[3], clark.I_abc_Y[3]);

      connect(VDC, vDC_Controller.u) annotation (Line(points={{-166,74},{-110,74},{-110,60},{-65.54,60}}, color={0,0,127}));
      connect(VDC, p_Controller.Vdc) annotation (Line(points={{-166,74},{-110,74},{-110,38.76},{-66.24,38.76}}, color={0,0,127}));
      connect(V_abc_D[1], clark.V_abc_D[1]);
      connect(V_abc_D[2], clark.V_abc_D[2]);
      connect(V_abc_D[3], clark.V_abc_D[3]);
      connect(I_abc_D[1], clark.I_abc_D[1]);
      connect(I_abc_D[2], clark.I_abc_D[2]);
      connect(I_abc_D[3], clark.I_abc_D[3]);
      connect(VDC, dQToABC.vdc) annotation (Line(points={{-166,74},{100,74},{100,25},{105.8,25}}, color={0,0,127}));
      connect(dQToABC.varef, Varef) annotation (Line(points={{133,22.8},{138,22.8},{138,48},{152,48}}, color={0,0,127}));
      connect(dQToABC.vbref, Vbref) annotation (Line(points={{133,18.2},{140,18.2},{140,6},{152,6}}, color={0,0,127}));
      connect(dQToABC.vcref, Vcref) annotation (Line(points={{133,13.6},{136,13.6},{136,-30},{152,-30}}, color={0,0,127}));
      connect(choiceController.y, idqRef_Limiter.Id_ref_in) annotation (Line(points={{24.6,46},{30,46},{30,61.4},{39.6,61.4}}, color={0,0,127}));
      connect(idqRef_Limiter.Id_ref_out, innerControl.Id_ref) annotation (Line(points={{63,62},{74,62},{74,42},{42,42},{42,26.4},{50.04,26.4}}, color={0,0,127}));
      connect(choiceQVAC_Controller.y, idqRef_Limiter.Iq_ref_in) annotation (Line(points={{24.7,26},{34,26},{34,53.6},{39.6,53.6}}, color={0,0,127}));
      connect(idqRef_Limiter.Iq_ref_out, innerControl.Iqref) annotation (Line(points={{63,54.2},{68,54.2},{68,46},{36,46},{36,-2.4},{49.68,-2.4}}, color={0,0,127}));
      connect(signalCalculation.V_ac_Grid, filter.u) annotation (Line(points={{-77.3,10.32},{-68,10.32},{-68,26},{-43.2,26}}, color={0,0,127}));
      connect(filter.y, power_Current.VacGrid) annotation (Line(points={{-29.4,26},{-26,26},{-26,46.48},{-24.1,46.48}}, color={0,0,127}));
      annotation (Diagram(coordinateSystem(extent={{-140,-100},{140,100}}, preserveAspectRatio=false)), Icon(coordinateSystem(extent={{-140,-100},{140,100}})));
    end UpperLevel;

    package dq_trnsfrmns
      model park_V " park_V block "

        Modelica.Blocks.Interfaces.RealInput Va annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
        Modelica.Blocks.Interfaces.RealInput Vb annotation (Placement(transformation(extent={{-140,-100},{-100,-60}})));
        Modelica.Blocks.Interfaces.RealInput theta annotation (Placement(transformation(
              extent={{20,-20},{-20,20}},
              rotation=-90,
              origin={0,-120})));
        Modelica.Blocks.Interfaces.RealOutput Vq annotation (Placement(transformation(extent={{100,-90},{120,-70}})));
        Modelica.Blocks.Interfaces.RealOutput Vd annotation (Placement(transformation(extent={{100,70},{120,90}})));
      equation

        Vd = Va*sin(theta) - Vb*cos(theta);
        Vq = Va*cos(theta) + Vb*sin(theta);

        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics));
      end park_V;

      model park_I " park_I block"

        Modelica.Blocks.Interfaces.RealInput Ia annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
        Modelica.Blocks.Interfaces.RealInput Ib annotation (Placement(transformation(extent={{-140,-100},{-100,-60}})));
        Modelica.Blocks.Interfaces.RealOutput Id annotation (Placement(transformation(extent={{100,68},{120,88}})));
        Modelica.Blocks.Interfaces.RealOutput Iq annotation (Placement(transformation(extent={{100,-88},{120,-68}})));
        Modelica.Blocks.Interfaces.RealInput theta annotation (Placement(transformation(
              extent={{20,-20},{-20,20}},
              rotation=-90,
              origin={0,-120})));

      equation
        Id = Ia*sin(theta) - Ib*cos(theta);
        Iq = Ia*cos(theta) + Ib*sin(theta);

        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics));
      end park_I;

      model dq_transfo " full dq_transfo that inclued park_V and park_I"

        Modelica.Blocks.Interfaces.RealInput V_aplha_Y annotation (Placement(transformation(extent={{-140,70},{-100,110}})));
        Modelica.Blocks.Interfaces.RealInput V_beta_Y annotation (Placement(transformation(extent={{-140,10},{-100,50}})));
        Modelica.Blocks.Interfaces.RealInput I_alpha_D annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={-120,-26})));
        Modelica.Blocks.Interfaces.RealInput I_beta_D annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={-120,-80})));
        Modelica.Blocks.Interfaces.RealInput theta annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=-90,
              origin={0,120})));
        Modelica.Blocks.Interfaces.RealOutput Vq annotation (Placement(transformation(extent={{100,30},{120,50}})));
        Modelica.Blocks.Interfaces.RealOutput Vd annotation (Placement(transformation(extent={{100,80},{120,100}})));
        Modelica.Blocks.Interfaces.RealOutput Id annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={110,-18})));
        Modelica.Blocks.Interfaces.RealOutput Iq annotation (Placement(transformation(
              extent={{10,-10},{-10,10}},
              rotation=180,
              origin={110,-80})));
        park_V park_V1 annotation (Placement(transformation(extent={{-26,60},{-6,80}})));
        park_I park_I1 annotation (Placement(transformation(extent={{-4,-44},{16,-24}})));
      equation

        connect(park_V1.Va, V_aplha_Y) annotation (Line(
            points={{-28,78},{-64,78},{-64,90},{-120,90}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(V_beta_Y, park_V1.Vb) annotation (Line(
            points={{-120,30},{-74,30},{-74,62},{-28,62}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(I_alpha_D, park_I1.Ia) annotation (Line(
            points={{-120,-26},{-6,-26}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(I_beta_D, park_I1.Ib) annotation (Line(
            points={{-120,-80},{-64,-80},{-64,-42},{-6,-42}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(theta, park_V1.theta) annotation (Line(
            points={{0,120},{0,40},{-16,40},{-16,58}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(park_I1.theta, park_V1.theta) annotation (Line(
            points={{6,-46},{6,-52},{38,-52},{38,52},{0,52},{0,40},{-16,40},{-16,58}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(park_V1.Vd, Vd) annotation (Line(
            points={{-5,78},{49.5,78},{49.5,90},{110,90}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(park_V1.Vq, Vq) annotation (Line(
            points={{-5,62},{76,62},{76,40},{110,40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(park_I1.Id, Id) annotation (Line(
            points={{17,-26.2},{60.5,-26.2},{60.5,-18},{110,-18}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(park_I1.Iq, Iq) annotation (Line(
            points={{17,-41.8},{78,-41.8},{78,-80},{110,-80}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics));
      end dq_transfo;

      package Example
        model dq_transfo_test
          Modelica.Blocks.Sources.Constant V_alpha_Y(k=8) annotation (Placement(transformation(extent={{-60,26},{-40,46}})));
          Modelica.Blocks.Sources.Constant theta(k=32) annotation (Placement(transformation(extent={{-10,46},{10,66}})));
          dq_transfo dq_transfo1 annotation (Placement(transformation(extent={{36,-30},{56,-10}})));
          Modelica.Blocks.Sources.Constant V_alpha_Y1(k=5) annotation (Placement(transformation(extent={{-86,-8},{-66,12}})));
          Modelica.Blocks.Sources.Constant V_alpha_Y2(k=2) annotation (Placement(transformation(extent={{-80,-40},{-60,-20}})));
          Modelica.Blocks.Sources.Constant V_alpha_Y3(k=7) annotation (Placement(transformation(extent={{-58,-84},{-38,-64}})));
          Modelica.Blocks.Interfaces.RealOutput Vd annotation (Placement(transformation(extent={{100,64},{120,84}})));
          Modelica.Blocks.Interfaces.RealOutput Vq annotation (Placement(transformation(extent={{100,12},{120,32}})));
          Modelica.Blocks.Interfaces.RealOutput Id annotation (Placement(transformation(extent={{100,-50},{120,-30}})));
          Modelica.Blocks.Interfaces.RealOutput Iq annotation (Placement(transformation(extent={{100,-94},{120,-74}})));
        equation

          connect(dq_transfo1.Vd, Vd) annotation (Line(
              points={{57,-11},{64,-11},{64,74},{110,74}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(dq_transfo1.Vq, Vq) annotation (Line(
              points={{57,-16},{70,-16},{70,-14},{82,-14},{82,22},{110,22}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(dq_transfo1.Id, Id) annotation (Line(
              points={{57,-21.8},{84,-21.8},{84,-40},{110,-40}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(dq_transfo1.Iq, Iq) annotation (Line(
              points={{57,-28},{62,-28},{62,-84},{110,-84}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(theta.y, dq_transfo1.theta) annotation (Line(
              points={{11,56},{46,56},{46,-8}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(V_alpha_Y.y, dq_transfo1.V_aplha_Y) annotation (Line(
              points={{-39,36},{-2,36},{-2,-11},{34,-11}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(V_alpha_Y1.y, dq_transfo1.V_beta_Y) annotation (Line(
              points={{-65,2},{-14,2},{-14,-17},{34,-17}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(V_alpha_Y2.y, dq_transfo1.I_alpha_D) annotation (Line(
              points={{-59,-30},{-38,-30},{-38,-22.6},{34,-22.6}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(V_alpha_Y3.y, dq_transfo1.I_beta_D) annotation (Line(
              points={{-37,-74},{-24,-74},{-24,-72},{10,-72},{10,-28},{34,-28}},
              color={0,0,127},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics));
        end dq_transfo_test;

        model Park_v_test
          Modelica.Blocks.Sources.Constant V_alpha_Y(k=2) annotation (Placement(transformation(extent={{-76,12},{-56,32}})));
          Modelica.Blocks.Sources.Constant V_alpha_Y1(k=6) annotation (Placement(transformation(extent={{-68,-28},{-48,-8}})));
          Modelica.Blocks.Sources.Constant V_alpha_Y2(k=45) annotation (Placement(transformation(extent={{-38,-48},{-18,-28}})));
          park_V park_V1 annotation (Placement(transformation(extent={{-18,4},{2,24}})));
        equation
          connect(V_alpha_Y2.y, park_V1.theta) annotation (Line(
              points={{-17,-38},{-8,-38},{-8,2}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(V_alpha_Y.y, park_V1.Va) annotation (Line(
              points={{-55,22},{-20,22}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(V_alpha_Y1.y, park_V1.Vb) annotation (Line(
              points={{-47,-18},{-34,-18},{-34,6},{-20,6}},
              color={0,0,127},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics));
        end Park_v_test;

        model park_I_test
          Modelica.Blocks.Sources.Constant V_alpha_Y(k=2) annotation (Placement(transformation(extent={{-48,22},{-28,42}})));
          Modelica.Blocks.Sources.Constant V_alpha_Y1(k=4) annotation (Placement(transformation(extent={{-54,-18},{-34,2}})));
          Modelica.Blocks.Sources.Constant V_alpha_Y2(k=30) annotation (Placement(transformation(extent={{-26,-38},{-6,-18}})));
          park_V park_V1 annotation (Placement(transformation(extent={{-4,24},{16,44}})));
        equation
          connect(V_alpha_Y.y, park_V1.Va) annotation (Line(
              points={{-27,32},{-18,32},{-18,42},{-6,42}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(V_alpha_Y1.y, park_V1.Vb) annotation (Line(
              points={{-33,-8},{-20,-8},{-20,26},{-6,26}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(V_alpha_Y2.y, park_V1.theta) annotation (Line(
              points={{-5,-28},{6,-28},{6,22}},
              color={0,0,127},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics));
        end park_I_test;

        model test
          Modelica.Blocks.Math.UnitConversions.From_deg from_deg annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=0,
                origin={0,8})));
          Modelica.Blocks.Sources.Constant const(k=45) annotation (Placement(transformation(extent={{-90,38},{-70,58}})));
          Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{100,-12},{120,8}})));
        equation
          connect(from_deg.y, y) annotation (Line(
              points={{11,8},{58,8},{58,-2},{110,-2}},
              color={0,0,127},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics));
        end test;

        model dq_transfo_step
          Modelica.Blocks.Sources.Step V_alpha_Y(
            height=0.7,
            offset=0.1,
            startTime=0.005) annotation (Placement(transformation(extent={{-126,44},{-106,64}})));
          Modelica.Blocks.Sources.Constant theta(k=32) annotation (Placement(transformation(extent={{-68,56},{-48,76}})));
          dq_transfo dq_transfo1 annotation (Placement(transformation(extent={{36,-30},{56,-10}})));
          Modelica.Blocks.Sources.Step V_beta_Y(
            height=0.5,
            offset=0.1,
            startTime=0.005) annotation (Placement(transformation(extent={{-114,-30},{-94,-10}})));
          Modelica.Blocks.Sources.Step V_alpha_Y3(
            height=0.6,
            offset=0.1,
            startTime=0.005) annotation (Placement(transformation(extent={{-66,-112},{-46,-92}})));
          Modelica.Blocks.Interfaces.RealOutput Vd annotation (Placement(transformation(extent={{100,62},{120,82}})));
          Modelica.Blocks.Interfaces.RealOutput Vq annotation (Placement(transformation(extent={{100,12},{120,32}})));
          Modelica.Blocks.Interfaces.RealOutput Id annotation (Placement(transformation(extent={{100,-50},{120,-30}})));
          Modelica.Blocks.Interfaces.RealOutput Iq annotation (Placement(transformation(extent={{100,-94},{120,-74}})));
          Modelica.Blocks.Sources.Step I_alpha_D(
            height=0.4,
            offset=0.1,
            startTime=0.005) annotation (Placement(transformation(extent={{-76,-64},{-56,-44}})));
        equation

          connect(theta.y, dq_transfo1.theta) annotation (Line(
              points={{-47,66},{-42,66},{-42,54},{46,54},{46,-8}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(V_alpha_Y.y, dq_transfo1.V_aplha_Y) annotation (Line(
              points={{-105,54},{-80,54},{-80,-11},{34,-11}},
              color={0,0,127},
              smooth=Smooth.None));

          connect(V_beta_Y.y, dq_transfo1.V_beta_Y) annotation (Line(
              points={{-93,-20},{-46,-20},{-46,-17},{34,-17}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(V_alpha_Y3.y, dq_transfo1.I_beta_D) annotation (Line(
              points={{-45,-102},{-22,-102},{-22,-28},{34,-28}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(dq_transfo1.Vd, Vd) annotation (Line(
              points={{57,-11},{64,-11},{64,74},{98,74},{98,72},{110,72}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(dq_transfo1.Vq, Vq) annotation (Line(
              points={{57,-16},{70,-16},{70,-14},{82,-14},{82,22},{110,22}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(dq_transfo1.Id, Id) annotation (Line(
              points={{57,-21.8},{84,-21.8},{84,-40},{110,-40}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(dq_transfo1.Iq, Iq) annotation (Line(
              points={{57,-28},{62,-28},{62,-84},{110,-84}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(I_alpha_D.y, dq_transfo1.I_alpha_D) annotation (Line(
              points={{-55,-54},{-46,-54},{-46,-30},{-34,-30},{-34,-22.6},{34,-22.6}},
              color={0,0,127},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics));
        end dq_transfo_step;
      end Example;
    end dq_trnsfrmns;

    model PU_Calculation
      parameter Real VmP=318992.28125;
      parameter Real VmS=220235.5;
      parameter Real ImP=505.628;
      parameter Real ImS=700.628;

      Modelica.Blocks.Interfaces.RealInput V_abc_Y[3]
        "Va, Vb and Vc in the primary side of Transformar"                                               annotation (Placement(transformation(extent={{-144,58},{-104,98}})));
      Modelica.Blocks.Interfaces.RealInput I_abc_Y[3]
        "Ia, Ib and Ic in the primary side of Transformar"                                               annotation (Placement(transformation(extent={{-144,10},{-104,50}})));
      Modelica.Blocks.Interfaces.RealInput V_abc_D[3] annotation (Placement(transformation(extent={{-144,-52},{-104,-12}})));
      Modelica.Blocks.Interfaces.RealInput I_abc_D[3] annotation (Placement(transformation(extent={{-144,-100},{-104,-60}})));
      Modelica.Blocks.Interfaces.RealOutput V_ab_Y[3] annotation (Placement(transformation(extent={{100,68},{120,88}})));
      Modelica.Blocks.Interfaces.RealOutput I_ab_Y[3] annotation (Placement(transformation(extent={{100,18},{120,38}})));
      Modelica.Blocks.Interfaces.RealOutput V_ab_D[3] annotation (Placement(transformation(extent={{100,-36},{120,-16}})));
      Modelica.Blocks.Interfaces.RealOutput I_ab_D[3] annotation (Placement(transformation(extent={{100,-86},{120,-66}})));
    equation
      V_ab_Y[:] = V_abc_Y[:]/VmP;
      I_ab_Y[:] = I_abc_Y[:]/ImP;
      V_ab_D[:] = V_abc_D[:]/VmS;
      I_ab_D[:] = I_abc_D[:]/ImS;
    end PU_Calculation;






    model VSC_MMC_AVM
      parameter Real Rated_Power=1000 "MVA";
      parameter Real AC_Prim_voltage=400 "kVRMSLL";
      parameter Real AC_Secon_voltage=320 "kVRMSLL";
      parameter Real freq=50 "Hz";
      parameter Real VDC=640*1000 "DC pole to pole voltage";
      parameter Real xtr=0.18 "Transformer Reactance (p.u.)";
      parameter Real rtr=0.001 "Transformer Resistance (p.u.)";
      parameter Real Larm=0.15 "MMC arm inductace (p.u.)";
      parameter Real EC=40 "Capacitor energy in each Submodule (kJ/MVA)";
      parameter Real Nsm=400 "Number of submodule per arm";
      parameter Real Rloss=0.001 "Conduction losses of each IGBT/diode";
      parameter Real Scc=10000;
      parameter Real VDC_ref=1 "VDC reference value";
      parameter Real KIVDC=293.8775510204082 "Integral gain of VDC controller";
      parameter Real KPVDC=9.6 "Proportional gain of VDC controller";
      parameter Real init_value_OVdc=0
        "Initial Output of Integrator of VDC controller";
      parameter Real Vdc_step=0 "Step (Height) value of VDC in the reference";
      parameter Real t_VdcStep=0 "Start Time of the step";
      parameter Real KIP=30 "Integral Gain of Active Power Regulator";
      parameter Real KPP=0 "Proportional gain of Active Power Regulator";
      parameter Real P_ref=1 "Active power reference (p.u.)";
      parameter Real P_step=0.01
        "Step (Height) value of Active Power, in the reference";
      parameter Real start_time_P=3 "Start Time of the step";
      parameter Real init_P=0 "Initial Output of Integrator of P controller";
      parameter Real P_Vdc_Kp=10 "Gain of Deadband control";
      parameter Real KIQ=30 "Integral gain of Q controller";
      parameter Real KPQ=0 "Proportional gain of P controller";
      parameter Real Q_ref=0 "Reference value of Reactive Power (p.u.)";
      parameter Real Q_step=0
        "Step (Height) value of Reactive Power, in the reference";
      parameter Real Qinit_value=0
        "Initial Output of Integrator of Q controller";
      parameter Real start_time_Q=0 "Start Time of the step";
      parameter Real Vac_ref=1 "Reference AC voltage (p.u.)";
      parameter Real Vac_step=0 "Step (Height) value of VAC, in the reference";
      parameter Real Vac_init=0
        "Initial Output of Integrator of VAC controller";
      parameter Real Vac_step_time=0 "Start Time of the step";
      parameter Real KIVAC=30 "Integral Gain of VAC controller";
      parameter Real KPVAC=0 "Proportional Gain of VAC contoller";
      parameter Integer Mode=1 annotation (Dialog(group="ControllerChoice"), choices(choice=1
            "Active Power Control",                                                                                   choice=0
            "DC Voltage Control"));
      parameter Real Ictrl_KP=0.48701412586119974;
      parameter Real Ictrl_KI=149.08595689628566;
      parameter Real Lac_eq_pu=0.255;
      parameter Real initvd=0;
      parameter Real initvq=0;
      parameter Integer C_QVAC=1 annotation (Dialog(group="ControllerChoice"), choices(choice=1
            "Reactive Power Control",                                                                                     choice=0
            "VAC Control"));
      parameter Real Freq=50;
      parameter Real time_step=0.00001;
      parameter Real I_lim=1.1;
      parameter Real Id_lim=1.1;
      parameter Real Iq_lim=0.5;
      parameter Integer Prio annotation (Dialog(group="Idqlimiter"), choices(choice=1
            "Active Power",                                                                           choice=0
            "Reactive Power"));
      import Modelica.Constants.pi;
      parameter Real initPLL=0.196350;
      parameter Real KpPLL=80;
      parameter Real KiPLL=1600;
      parameter Real Vb_P=(AC_Prim_voltage*1000/sqrt(3))*sqrt(2);
      parameter Real Vb_S=(AC_Secon_voltage*1000/sqrt(3))*sqrt(2);
      parameter Real Ib_P=((Rated_Power*1000/AC_Prim_voltage)/sqrt(3))*sqrt(2);
      parameter Real Ib_S=((Rated_Power*1000/AC_Secon_voltage)/sqrt(3))*sqrt(2);
      parameter Real Zb_S=(AC_Secon_voltage^2)/Rated_Power;
      parameter Real L_arm_H=Larm*(Zb_S/(2*pi*freq));
      parameter Real AVM_Leq_AC=L_arm_H/2;
      parameter Real AVM_Leq_DC=(L_arm_H/3)*2;
      parameter Real AVM_Req=Rloss*Nsm*2/3;
      parameter Real Csm_C=(2*Rated_Power*10^6*EC*0.001)/((6/Nsm)*VDC^2);
      parameter Real AVM_Ceq_C=6*(Csm_C/Nsm);
      parameter Real Vcinit;
      parameter Real Ilinit;

      AVM aVM(
        Lac=AVM_Leq_AC,
        Rdc=AVM_Req,
        Ceq=AVM_Ceq_C,
        Ldc=AVM_Leq_DC,
        V_C_init=Vcinit,
        I_L_init=Ilinit) annotation (Placement(transformation(extent={{-2,40},{32,74}})));
      ThreePhsaeYDtransformer threePhsaeYDtransformer(
        R_trans_prim=0.14400000000000002,
        L_trans_prim=0.08250592249883855,
        R_trans_secon=0.010239999999999999,
        L_trans_secon=0.005867087822139628) annotation (Placement(transformation(extent={{-86,58},{-66,78}})));
      Modelica.Electrical.MultiPhase.Sensors.CurrentSensor C_P annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-118,68})));
      Modelica.Electrical.MultiPhase.Sensors.CurrentSensor C_S annotation (Placement(transformation(extent={{-44,58},{-24,78}})));
      Modelica.Electrical.MultiPhase.Sensors.PotentialSensor P_P annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-98,38})));
      Modelica.Electrical.MultiPhase.Sensors.PotentialSensor P_S annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-40,42})));
      PU_Calculation pU_Calculation(
        VmP=Vb_P,
        VmS=Vb_S,
        ImP=Ib_P,
        ImS=Ib_S) annotation (Placement(transformation(extent={{-36,-68},{-16,-48}})));
      UpperLevel upperLevel(
        VDC_ref=VDC_ref,
        KIVDC=KIVDC,
        KPVDC=KPVDC,
        init_value_OVdc=init_value_OVdc,
        Vdc_step=Vdc_step,
        t_VdcStep=t_VdcStep,
        KIP=KIP,
        KPP=KPP,
        P_ref=P_ref,
        P_step=P_step,
        start_time_P=start_time_P,
        init_P=init_P,
        P_Vdc_Kp=P_Vdc_Kp,
        KIQ=KIQ,
        KPQ=KPQ,
        Q_ref=Q_ref,
        Q_step=Q_step,
        Qinit_value=Qinit_value,
        start_time_Q=start_time_Q,
        Vac_ref=Vac_ref,
        Vac_step=Vac_step,
        Vac_init=Vac_init,
        Vac_step_time=Vac_step_time,
        KIVAC=KIVAC,
        KPVAC=KPVAC,
        Ictrl_KP=Ictrl_KP,
        Ictrl_KI=Ictrl_KI,
        Lac_eq_pu=Lac_eq_pu,
        initvd=initvd,
        initvq=initvq,
        Freq=Freq,
        time_step=time_step,
        initPLL=initPLL,
        KpPLL=KpPLL,
        KiPLL=KiPLL,
        Mode=Mode,
        C_QVAC=C_QVAC,
        I_lim=I_lim,
        Id_lim=Id_lim,
        Iq_lim=Iq_lim,
        Prio=Prio) annotation (Placement(transformation(extent={{18,-52},{46,-32}})));
      Modelica.Blocks.Math.Gain gain(k=1/VDC) annotation (Placement(transformation(extent={{46,34},{58,46}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin P annotation (Placement(transformation(extent={{114,44},{134,64}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin N annotation (Placement(transformation(extent={{114,-52},{134,-32}})));
      Modelica.Electrical.MultiPhase.Interfaces.PositivePlug P_Plug annotation (Placement(transformation(extent={{-164,-10},{-144,10}})));
    equation
      connect(P_P.phi[1], pU_Calculation.V_abc_Y[1]) annotation (Line(points={{-98,27},
              {-50,27},{-50,-51.5333},{-38.4,-51.5333}},                                                                               color={0,0,127}));
      connect(P_P.phi[2], pU_Calculation.V_abc_Y[2]);
      connect(P_P.phi[3], pU_Calculation.V_abc_Y[3]);
      connect(C_P.i[1], pU_Calculation.I_abc_Y[1]) annotation (Line(points={{-118,57},
              {-110,57},{-110,-56.3333},{-38.4,-56.3333}},                                                                                   color={0,0,127}));
      connect(C_P.i[2], pU_Calculation.I_abc_Y[2]);
      connect(C_P.i[3], pU_Calculation.I_abc_Y[3]);

      connect(P_S.phi[1], pU_Calculation.V_abc_D[1]) annotation (Line(points={{-40,31},
              {-58,31},{-58,-62.5333},{-38.4,-62.5333}},                                                                               color={0,0,127}));
      connect(P_S.phi[2], pU_Calculation.V_abc_D[2]);
      connect(P_S.phi[3], pU_Calculation.V_abc_D[3]);

      connect(C_S.i[1], pU_Calculation.I_abc_D[1]) annotation (Line(points={{-34,57},
              {-56,57},{-56,48},{-72,48},{-72,-67.3333},{-38.4,-67.3333}},                                                   color={0,0,127}));
      connect(C_S.i[2], pU_Calculation.I_abc_D[2]);
      connect(C_S.i[3], pU_Calculation.I_abc_D[3]);
      connect(pU_Calculation.V_ab_Y[1], upperLevel.V_abc_Y[1]) annotation (Line(points={{-15,
              -50.8667},{-8,-50.8667},{-8,-39.7333},{15.4,-39.7333}},                                                                                color={0,0,127}));
      connect(pU_Calculation.V_ab_Y[2], upperLevel.V_abc_Y[2]);
      connect(pU_Calculation.V_ab_Y[3], upperLevel.V_abc_Y[3]);
      connect(pU_Calculation.I_ab_Y[1], upperLevel.I_abc_Y[1]) annotation (Line(points={{-15,
              -55.8667},{-10,-55.8667},{-10,-43.7333},{15.6,-43.7333}},                                                                                color={0,0,127}));
      connect(pU_Calculation.I_ab_Y[2], upperLevel.I_abc_Y[2]);
      connect(pU_Calculation.I_ab_Y[3], upperLevel.I_abc_Y[3]);
      connect(pU_Calculation.V_ab_D[1], upperLevel.V_abc_D[1]) annotation (Line(points={{-15,
              -61.2667},{-4,-61.2667},{-4,-47.3333},{15.2,-47.3333}},                                                                                color={0,0,127}));
      connect(pU_Calculation.V_ab_D[2], upperLevel.V_abc_D[2]);
      connect(pU_Calculation.V_ab_D[3], upperLevel.V_abc_D[3]);
      connect(pU_Calculation.I_ab_D[1], upperLevel.I_abc_D[1]) annotation (Line(points={{-15,
              -66.2667},{-8,-66.2667},{-8,-51.3333},{15.2,-51.3333}},                                                                                color={0,0,127}));
      connect(pU_Calculation.I_ab_D[2], upperLevel.I_abc_D[2]);
      connect(pU_Calculation.I_ab_D[3], upperLevel.I_abc_D[3]);

      connect(threePhsaeYDtransformer.negativePlug, C_S.plug_p) annotation (Line(points={{-65.6,68},{-54.8,68},{-44,68}}, color={0,0,255}));
      connect(C_P.plug_n, threePhsaeYDtransformer.positivePlug) annotation (Line(points={{-108,68},{-86.4,68}}, color={0,0,255}));
      connect(threePhsaeYDtransformer.positivePlug, P_P.plug_p) annotation (Line(points={{-86.4,68},{-98,68},{-98,48}}, color={0,0,255}));
      connect(upperLevel.Varef, aVM.varef) annotation (Line(points={{47.2,-37.2},{56,-37.2},{56,6},{-20,6},{-20,50.71},{-8.29,50.71}}, color={0,0,127}));
      connect(upperLevel.Vbref, aVM.vbref) annotation (Line(points={{47.2,-41.4},{60,-41.4},{60,12},{-18,12},{-18,47.14},{-8.46,47.14}}, color={0,0,127}));
      connect(upperLevel.Vcref, aVM.vcref) annotation (Line(points={{47.2,-45},{70,-45},{70,26},{-16,26},{-16,43.74},{-8.46,43.74}}, color={0,0,127}));
      connect(aVM.VDC, gain.u) annotation (Line(points={{33.7,44.42},{38,44.42},{38,40},{44.8,40}}, color={0,0,127}));
      connect(gain.y, upperLevel.VDC) annotation (Line(points={{58.6,40},{88,40},{88,-14},{8,-14},{8,-34.6},{15.4,-34.6}}, color={0,0,127}));
      connect(aVM.P, P) annotation (Line(points={{32.68,66.52},{86,66.52},{86,54},{124,54}}, color={0,0,255}));
      connect(aVM.N, N) annotation (Line(points={{32.68,48.84},{98,48.84},{98,-42},{124,-42}}, color={0,0,255}));
      connect(C_P.plug_p, P_Plug) annotation (Line(points={{-128,68},{-136,68},{-136,0},{-154,0}}, color={0,0,255}));
      connect(C_S.plug_p, P_S.plug_p) annotation (Line(points={{-44,68},{-44,68},{-44,54},{-44,52},{-40,52}}, color={0,0,255}));
      connect(C_S.plug_n, aVM.AC) annotation (Line(points={{-24,68},{-16,68},{-16,57.34},{-2.68,57.34}}, color={0,0,255}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,-100},{120,100}})), Icon(coordinateSystem(extent={{-140,-100},{120,100}})));
    end VSC_MMC_AVM;


    model PI_Delay
      parameter Real init_value;
      parameter Real max;
      parameter Real min;
      parameter Real KI;
      parameter Real KP;
      parameter Real time_step=0.001;

      Modelica.Blocks.Continuous.LimIntegrator limIntegrator(
        k=1,
        outMax=1e5,
        initType=Modelica.Blocks.Types.Init.InitialOutput,
        y_start=init_value) annotation (Placement(transformation(extent={{-14,-10},{6,10}})));
      Modelica.Blocks.Interfaces.RealInput u annotation (Placement(transformation(extent={{-142,-20},{-102,20}})));
      Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Modelica.Blocks.Math.Add add(k2=+1) annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
      Modelica.Blocks.Math.Add add1(k1=+1, k2=-1) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={42,-38})));
      Modelica.Blocks.Math.Add add2(k1=KP, k2=KI) annotation (Placement(transformation(extent={{26,18},{42,34}})));
      Modelica.Blocks.Nonlinear.Limiter limiter(uMax=max, uMin=min) annotation (Placement(transformation(extent={{58,20},{74,36}})));
      Modelica.Blocks.Nonlinear.FixedDelay fixedDelay(delayTime=time_step) annotation (Placement(transformation(
            extent={{-7,-7},{7,7}},
            rotation=180,
            origin={-21,-37})));
    equation
      connect(add.y, limIntegrator.u) annotation (Line(points={{-49,0},{-16,0}}, color={0,0,127}));
      connect(u, add.u1) annotation (Line(points={{-122,0},{-106,0},{-92,0},{-92,6},{-72,6}}, color={0,0,127}));
      connect(add2.u1, add.u1) annotation (Line(points={{24.4,30.8},{-92,30.8},{-92,6},{-72,6}}, color={0,0,127}));
      connect(limIntegrator.y, add2.u2) annotation (Line(points={{7,0},{16,0},{16,21.2},{24.4,21.2}}, color={0,0,127}));
      connect(add2.y, limiter.u) annotation (Line(points={{42.8,26},{48,26},{48,28},{56.4,28}}, color={0,0,127}));
      connect(limiter.y, y) annotation (Line(points={{74.8,28},{88,28},{88,0},{110,0}}, color={0,0,127}));
      connect(add1.u2, limiter.u) annotation (Line(points={{54,-32},{58,-32},{58,-10},{48,-10},{48,28},{56.4,28}}, color={0,0,127}));
      connect(add1.u1, y) annotation (Line(points={{54,-44},{92,-44},{92,0},{110,0}}, color={0,0,127}));
      connect(fixedDelay.u, add1.y) annotation (Line(points={{-12.6,-37},{10.7,-37},{10.7,-38},{31,-38}}, color={0,0,127}));
      connect(fixedDelay.y, add.u2) annotation (Line(points={{-28.7,-37},{-90,-37},{-90,-6},{-72,-6}}, color={0,0,127}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end PI_Delay;

    model IdqRef_Limiter
      parameter Real I_lim=1.1;
      parameter Real Id_lim=1.1;
      parameter Real Iq_lim=0.5;
      parameter Integer Prio annotation (Dialog(group="Idqlimiter"), choices(choice=1
            "Active Power",                                                                           choice=0
            "Reactive Power"));

      Modelica.Blocks.Interfaces.RealInput Id_ref_in annotation (Placement(transformation(extent={{-144,14},{-104,54}})));
      Modelica.Blocks.Interfaces.RealInput Iq_ref_in annotation (Placement(transformation(extent={{-144,-64},{-104,-24}})));
      Modelica.Blocks.Nonlinear.VariableLimiter vLimiter annotation (Placement(transformation(extent={{-30,24},{-10,44}})));
      Modelica.Blocks.Nonlinear.VariableLimiter vLimiter1 annotation (Placement(transformation(extent={{-26,-52},{-6,-32}})));
      Modelica.Blocks.Interfaces.RealOutput Id_ref_out annotation (Placement(transformation(extent={{100,30},{120,50}})));
      Modelica.Blocks.Interfaces.RealOutput Iq_ref_out annotation (Placement(transformation(extent={{100,-48},{120,-28}})));
      F1_IDQREF f1_IDQREF(
        I_lim=I_lim,
        Id_lim=Id_lim,
        Iq_lim=Iq_lim,
        Prio=Prio) annotation (Placement(transformation(extent={{-80,44},{-60,64}})));
      F2_IDQREF f2_IDQREF(
        I_lim=I_lim,
        Id_lim=Id_lim,
        Iq_lim=Iq_lim,
        Prio=Prio) annotation (Placement(transformation(extent={{-76,-28},{-56,-8}})));
    equation

      connect(vLimiter.y, Id_ref_out) annotation (Line(points={{-9,34},{46,34},{46,40},{110,40}}, color={0,0,127}));
      connect(vLimiter1.y, Iq_ref_out) annotation (Line(points={{-5,-42},{48,-42},{48,-38},{110,-38}}, color={0,0,127}));
      connect(Id_ref_in, vLimiter.u) annotation (Line(points={{-124,34},{-32,34},{-32,34}}, color={0,0,127}));
      connect(f1_IDQREF.y, vLimiter.limit1) annotation (Line(points={{-59,59.4},{-46,59.4},{-46,42},{-32,42}}, color={0,0,127}));
      connect(f1_IDQREF.y1, vLimiter.limit2) annotation (Line(points={{-59,50.6},{-50,50.6},{-50,26},{-32,26}}, color={0,0,127}));
      connect(Iq_ref_in, vLimiter1.u) annotation (Line(points={{-124,-44},{-28,-44},{-28,-42}}, color={0,0,127}));
      connect(f2_IDQREF.y, vLimiter1.limit1) annotation (Line(points={{-55,-12.6},{-36,-12.6},{-36,-34},{-28,-34}}, color={0,0,127}));
      connect(f2_IDQREF.y1, vLimiter1.limit2) annotation (Line(points={{-55,-21.4},{-44,-21.4},{-44,-50},{-28,-50}}, color={0,0,127}));
      connect(vLimiter1.y, f1_IDQREF.u) annotation (Line(points={{-5,-42},{8,-42},{8,80},{-94,80},{-94,54},{-82.2,54}}, color={0,0,127}));
      connect(vLimiter.y, f2_IDQREF.u) annotation (Line(points={{-9,34},{-4,34},{-4,4},{-88,4},{-88,-18},{-78.2,-18}}, color={0,0,127}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end IdqRef_Limiter;

    model F1_IDQREF
      parameter Real I_lim=1.1;
      parameter Real Id_lim=1.1;
      parameter Real Iq_lim=0.5;
      Real out1;
      parameter Integer Prio annotation (Dialog(group="Idqlimiter"), choices(choice=1
            "Active Power",                                                                           choice=0
            "Reactive Power"));
      Modelica.Blocks.Interfaces.RealInput u annotation (Placement(transformation(extent={{-142,-20},{-102,20}})));
      Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{100,44},{120,64}})));
      Modelica.Blocks.Interfaces.RealOutput y1 annotation (Placement(transformation(extent={{100,-44},{120,-24}})));
    equation
      out1 = sqrt(I_lim*I_lim - u*u);

      if Prio == 1 then
        y = Id_lim;
        y1 = -Id_lim;
      else
        y = out1;
        y1 = -out1;
      end if;

    end F1_IDQREF;

    model F2_IDQREF
      parameter Real I_lim=1.1;
      parameter Real Id_lim=1.1;
      parameter Real Iq_lim=0.5;
      Real out2;
      parameter Integer Prio annotation (Dialog(group="Idqlimiter"), choices(choice=1
            "Active Power",                                                                           choice=0
            "Reactive Power"));
      Modelica.Blocks.Interfaces.RealInput u annotation (Placement(transformation(extent={{-142,-20},{-102,20}})));
      Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{100,44},{120,64}})));
      Modelica.Blocks.Interfaces.RealOutput y1 annotation (Placement(transformation(extent={{100,-44},{120,-24}})));
    equation
      out2 = sqrt(I_lim*I_lim - u*u);

      if Prio == 0 then
        y = Iq_lim;
        y1 = -Iq_lim;
      else
        y = out2;
        y1 = -out2;
      end if;

    end F2_IDQREF;

    model SignalCalculation

      Modelica.Blocks.Interfaces.RealInput V_al_Y annotation (Placement(transformation(extent={{-142,54},{-102,94}})));
      Modelica.Blocks.Interfaces.RealInput V_be_Y annotation (Placement(transformation(extent={{-144,4},{-104,44}})));
      Modelica.Blocks.Interfaces.RealInput I_al_Y annotation (Placement(transformation(extent={{-146,-42},{-106,-2}})));
      Modelica.Blocks.Interfaces.RealInput I_be_Y annotation (Placement(transformation(extent={{-144,-88},{-104,-48}})));
      Modelica.Blocks.Continuous.Filter filter(
        f_cut=140,
        filterType=Modelica.Blocks.Types.FilterType.LowPass,
        analogFilter=Modelica.Blocks.Types.AnalogFilter.CriticalDamping) annotation (Placement(transformation(extent={{12,44},{32,64}})));
      Modelica.Blocks.Continuous.Filter filter1(f_cut=140,filterType=Modelica.Blocks.Types.FilterType.LowPass) annotation (Placement(transformation(extent={{10,-8},{30,12}})));
      Modelica.Blocks.Continuous.Filter filter2(f_cut=140,filterType=Modelica.Blocks.Types.FilterType.LowPass) annotation (Placement(transformation(extent={{12,-56},{32,-36}})));
      Modelica.Blocks.Interfaces.RealOutput P annotation (Placement(transformation(extent={{100,46},{120,66}})));
      Modelica.Blocks.Interfaces.RealOutput Q annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Modelica.Blocks.Interfaces.RealOutput V_ac_Grid annotation (Placement(transformation(extent={{100,-56},{120,-36}})));
    equation
      filter.u = V_al_Y*I_al_Y + V_be_Y*I_be_Y;
      filter1.u = -V_al_Y*I_be_Y + V_be_Y*I_al_Y;
      filter2.u = sqrt(V_al_Y*V_al_Y + V_be_Y*V_be_Y);
      connect(filter.y, P) annotation (Line(points={{33,54},{67.5,54},{67.5,56},{110,56}}, color={0,0,127}));
      connect(filter1.y, Q) annotation (Line(points={{31,2},{66,2},{66,0},{110,0}}, color={0,0,127}));
      connect(filter2.y, V_ac_Grid) annotation (Line(points={{33,-46},{68,-46},{68,-46},{110,-46}}, color={0,0,127}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end SignalCalculation;

    model SignalCalculationNoFilter

      Modelica.Blocks.Interfaces.RealInput V_al_Y annotation (Placement(transformation(extent={{-142,54},{-102,94}})));
      Modelica.Blocks.Interfaces.RealInput V_be_Y annotation (Placement(transformation(extent={{-144,4},{-104,44}})));
      Modelica.Blocks.Interfaces.RealInput I_al_Y annotation (Placement(transformation(extent={{-146,-42},{-106,-2}})));
      Modelica.Blocks.Interfaces.RealInput I_be_Y annotation (Placement(transformation(extent={{-144,-88},{-104,-48}})));
      Modelica.Blocks.Interfaces.RealOutput P annotation (Placement(transformation(extent={{100,46},{120,66}})));
      Modelica.Blocks.Interfaces.RealOutput Q annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Modelica.Blocks.Interfaces.RealOutput V_ac_Grid annotation (Placement(transformation(extent={{100,-56},{120,-36}})));
    equation
      P = V_al_Y*I_al_Y + V_be_Y*I_be_Y;
      Q = -V_al_Y*I_be_Y + V_be_Y*I_al_Y;
      V_ac_Grid = sqrt(V_al_Y*V_al_Y + V_be_Y*V_be_Y);
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
    end SignalCalculationNoFilter;
    annotation (Documentation(info="<html>
<p>
This package contains one equivalent source model, one three phase two winding transformer model 
and one complete VSC HVDC model with controls.
</p>
</html>"));
  end HVDC_Emtp;

  package Example

















    package EquivalentGenerator

      model Eq_Source_Test

        Modelica.Electrical.MultiPhase.Basic.Resistor resistor(R=fill(10, 3)) annotation (Placement(transformation(extent={{-34,26},{-14,46}})));
        Modelica.Electrical.Analog.Basic.Ground groundT2 annotation (Placement(transformation(extent={{30,-40},
                  {50,-20}},                                                                                              rotation=0)));
        Modelica.Electrical.MultiPhase.Basic.Star star1 annotation (Placement(transformation(extent={{-10,-10},
                  {10,10}},
              rotation=-90,
              origin={0,-10})));
        .VSCHVDC.HVDC_Emtp.Inductor inductor(L=fill(1, 3))
          annotation (Placement(transformation(extent={{14,26},{34,46}})));
        .VSCHVDC.HVDC_Emtp.EquivalentSource.Equivalent_Source equivalent_Source
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-60,12})));
      equation
        connect(resistor.plug_n, inductor.plug_p) annotation (Line(points={{-14,36},{14,
                36}},                                                                               color={0,0,255}));
        connect(equivalent_Source.positivePlug, resistor.plug_p)
          annotation (Line(points={{-60,22.4},{-60,36},{-34,36}}, color={0,0,255}));
        connect(star1.pin_n, groundT2.p)
          annotation (Line(points={{0,-20},{20,-20},{40,-20}}, color={0,0,255}));
        connect(inductor.plug_n, star1.plug_p) annotation (Line(points={{34,36},{44,36},
                {44,34},{44,0},{0,0}}, color={0,0,255}));
        connect(equivalent_Source.negativePlug, star1.plug_p) annotation (Line(points=
               {{-60,1.4},{-30,1.4},{-30,0},{0,0}}, color={0,0,255}));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})),experiment(
            StopTime=0.4,
            Tolerance=1e-4,
            __Dymola_fixedstepsize=0.0001,
            __Dymola_Algorithm="Dassl"),
          __Dymola_experimentSetupOutput);
      end Eq_Source_Test;
    end EquivalentGenerator;

    package Transformer
      "Three phase two winding transformer example with an equivalent source"

      model EqGen_Trans_Test

        Modelica.Electrical.MultiPhase.Basic.Resistor resistor(R=fill(10, 3)) annotation (Placement(transformation(extent={{-10,12},
                  {10,32}})));
        Modelica.Electrical.Analog.Basic.Ground groundT2 annotation (Placement(transformation(extent={{66,-48},
                  {86,-28}},                                                                                              rotation=0)));
        .VSCHVDC.HVDC_Emtp.Inductor inductor(
          L=fill(1, 3),
          i_init_a=0,
          i_init_b=0,
          i_init_c=0)
          annotation (Placement(transformation(extent={{32,12},{52,32}})));
        .VSCHVDC.HVDC_Emtp.EquivalentSource.Equivalent_Source equivalent_Source(
          ia0=0,
          ib0=0,
          ic0=0) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-58,-10})));
        HVDC_Emtp.ThreePhsaeYDtransformer threePhsaeYDtransformer(
          L_trans_prim=0.082,
          R_trans_secon=0.0102,
          L_trans_secon=0.005) annotation (Placement(transformation(extent={{-48,12},{
                  -28,32}})));
        Modelica.Electrical.MultiPhase.Basic.Star star2 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={76,-6})));
      equation
        connect(resistor.plug_n, inductor.plug_p) annotation (Line(points={{10,22},{20,
                22},{32,22}},                                                                        color={0,0,255}));
        connect(threePhsaeYDtransformer.negativePlug, resistor.plug_p) annotation (Line(points={{-27.6,
                22},{-10,22}},                                                                                        color={0,0,255}));
        connect(inductor.plug_n, star2.plug_p) annotation (Line(points={{52,22},{76,22},
                {76,4}},                                                                          color={0,0,255}));
        connect(star2.pin_n, groundT2.p) annotation (Line(points={{76,-16},{76,-16},{76,
                -28}},                                                                     color={0,0,255}));
        connect(threePhsaeYDtransformer.positivePlug, equivalent_Source.positivePlug) annotation (Line(points={{-48.4,
                22},{-58,22},{-58,0.4}},                                                                                                    color={0,0,255}));
        connect(equivalent_Source.negativePlug, star2.plug_p) annotation (Line(points={{-58,
                -20.6},{-58,-32},{-2,-32},{-2,4},{76,4}},                                                                                color={0,0,255}));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})),experiment(
            StopTime=0.4,
            Tolerance=1e-4,
            __Dymola_fixedstepsize=0.0001,
            __Dymola_Algorithm="Dassl"),
          __Dymola_experimentSetupOutput);
      end EqGen_Trans_Test;
    end Transformer;

    package VSC "Contains test system for different components of VSC "

      model PLLTest

        Modelica.Blocks.Sources.Cosine cosine(freqHz=50, amplitude=1) annotation (Placement(transformation(extent={{-92,44},
                  {-74,62}})));
        Modelica.Blocks.Sources.Cosine cosine1(
          freqHz=50,
          amplitude=1,
          phase=-2.0943951023932) annotation (Placement(transformation(extent={{-94,-24},
                  {-74,-4}})));
        HVDC_Emtp.PLL.PLL_Full pLL_Full(Freq=50, time_step=0.00001) annotation (Placement(transformation(extent={{14,-8},
                  {38,12}})));
        Modelica.Blocks.Math.Add add(k2=+1) annotation (Placement(transformation(extent={{-46,32},{-26,52}})));
        Modelica.Blocks.Sources.Step step(startTime=0.4, height=1) annotation (Placement(transformation(extent={{-94,10},
                  {-74,30}})));
      equation
        connect(cosine1.y, pLL_Full.vb) annotation (Line(points={{-73,-14},{-34,
                -14},{-34,2.6},{11.4,2.6}},                                                                  color={0,0,127}));
        connect(cosine.y, add.u1) annotation (Line(points={{-73.1,53},{-54,53},{
                -54,48},{-48,48}},                                                                 color={0,0,127}));
        connect(add.y, pLL_Full.va) annotation (Line(points={{-25,42},{-12,42},{
                -12,22},{-26,22},{-26,9.2},{10.8,9.2}},                                                                  color={0,0,127}));
        connect(step.y, add.u2) annotation (Line(points={{-73,20},{-56,20},{-56,
                36},{-48,36}},                                                                 color={0,0,127}));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})),experiment(
            StopTime=0.8,
            Tolerance=1e-5,
            __Dymola_fixedstepsize=0.0001,
            __Dymola_Algorithm="Rkfix4"),
          __Dymola_experimentSetupOutput);
      end PLLTest;

      model AVM_Test

        HVDC_Emtp.EquivalentSource.Equivalent_Source equivalent_Source(
          ia0=0,
          ib0=0,
          ic0=0) annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-68,-8})));
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=10) annotation (Placement(transformation(extent={{38,-2},
                  {58,18}})));
        Modelica.Electrical.MultiPhase.Basic.Star star1 annotation (Placement(transformation(extent={{-70,-44},
                  {-50,-24}})));
        Modelica.Electrical.Analog.Basic.Ground groundT2 annotation (Placement(transformation(extent={{-48,-56},
                  {-28,-36}},                                                                                            rotation=0)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=320*1000*2)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={82,-12})));
        Modelica.Electrical.Analog.Basic.Ground groundT1 annotation (Placement(transformation(extent={{72,-52},
                  {92,-32}},                                                                                             rotation=0)));
        HVDC_Emtp.VSC_MMC_AVM vSC_AVM(
          Vcinit=640000,
          Ilinit=1600,
          Mode=1,
          Prio=1)                     annotation (Placement(transformation(extent={{-14,-8},
                  {12,12}})));
      equation
        connect(equivalent_Source.negativePlug, star1.plug_p) annotation (Line(points={{-68,
                -18.6},{-70,-18.6},{-70,-34}},                                                                          color={0,0,255}));
        connect(constantVoltage.n, groundT1.p) annotation (Line(points={{82,-22},{82,-32}},color={0,0,255}));
        connect(vSC_AVM.P, resistor.p) annotation (Line(points={{12.4,7.4},{24.2,7.4},
                {24.2,8},{38,8}}, color={0,0,255}));
        connect(resistor.n, constantVoltage.p)
          annotation (Line(points={{58,8},{82,8},{82,-2}}, color={0,0,255}));
        connect(vSC_AVM.N, constantVoltage.n) annotation (Line(points={{12.4,-2.2},{48,
                -2.2},{48,-28},{82,-28},{82,-22}}, color={0,0,255}));
        connect(star1.pin_n, groundT2.p) annotation (Line(points={{-50,-34},{-44,-34},
                {-44,-36},{-38,-36}}, color={0,0,255}));
        connect(equivalent_Source.positivePlug, vSC_AVM.P_Plug) annotation (Line(
              points={{-68,2.4},{-42,2.4},{-42,2},{-15.4,2}}, color={0,0,255}));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}})),experiment(
            StopTime=0.8,
            Tolerance=1e-4,
            __Dymola_fixedstepsize=0.0001,
            __Dymola_Algorithm="Dassl"),
          __Dymola_experimentSetupOutput);
      end AVM_Test;

      model PITest_NoDelay
        Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{36,0},{56,20}})));
        Modelica.Blocks.Sources.Pulse pulse(
          startTime=0,
          nperiod=2,
          width=50,
          period=10,
          amplitude=-0.016,
          offset=0.008) annotation (Placement(transformation(extent={{-52,0},{-32,20}})));
        HVDC_Emtp.PI pI(
          init_value=0,
          KI=50,
          KP=10,
          max=0.8,
          min=-0.8) annotation (Placement(transformation(extent={{-8,0},{12,20}})));
      equation
        connect(pI.y, y) annotation (Line(points={{13,10},{46,10}}, color={0,0,127}));
        connect(pulse.y, pI.u) annotation (Line(points={{-31,10},{-10.2,10}}, color={0,0,127}));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
      end PITest_NoDelay;

      model PITest_WithDelay
        Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{36,0},{56,20}})));
        Modelica.Blocks.Sources.Pulse pulse(
          startTime=0,
          nperiod=2,
          width=50,
          period=10,
          amplitude=-0.016,
          offset=0.008) annotation (Placement(transformation(extent={{-52,0},{-32,20}})));
        HVDC_Emtp.PI_Delay pI_Delay(
          init_value=0,
          max=0.8,
          min=-0.8,
          KI=50,
          KP=10) annotation (Placement(transformation(extent={{-8,0},{12,20}})));
      equation
        connect(pulse.y, pI_Delay.u) annotation (Line(points={{-31,10},{-10.2,10},{-10.2,10}}, color={0,0,127}));
        connect(pI_Delay.y, y) annotation (Line(points={{13,10},{46,10},{46,10}}, color={0,0,127}));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
      end PITest_WithDelay;

      model TwoNodePowerSystem "Two Node Power system example"

        Modelica.Electrical.Analog.Basic.Resistor resistor(R=1.022) annotation (Placement(transformation(extent={{-4,50},{16,70}})));
        Modelica.Electrical.MultiPhase.Basic.Star star1 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-46,-10})));
        Modelica.Electrical.Analog.Basic.Ground groundT2 annotation (Placement(transformation(extent={{-54,-52},{-34,-32}}, rotation=0)));
        Modelica.Electrical.Analog.Basic.Ground groundT5 annotation (Placement(transformation(extent={{-18,22},{2,42}}, rotation=0)));
        HVDC_Emtp.EquivalentSource.Equivalent_Source equivalent_Source(
          ia0=0,
          ib0=0,
          ic0=0) annotation (Placement(transformation(extent={{-88,2},{-68,22}})));
        HVDC_Emtp.EquivalentSource.Equivalent_Source equivalent_Source1(
          ia0=0,
          ib0=0,
          ic0=0) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={70,10})));
        HVDC_Emtp.VSC_MMC_AVM vSC_MMC_AVMP(
          Mode=1,
          P_ref=1,
          Vcinit=642007,
          Ilinit=1557.3,
          Prio=0,
          time_step=0.00001,
          P_step=-0.5,
          initvq=0,
          C_QVAC=1,
          start_time_P=0.8,
          VDC_ref=1) annotation (Placement(transformation(extent={{-82,44},{-56,64}})));
        HVDC_Emtp.VSC_MMC_AVM vSC_MMC_AVMVDC(
          Mode=0,
          Vdc_step=0,
          t_VdcStep=0,
          Vcinit=639585,
          Ilinit=-1557.3,
          Prio=0,
          time_step=0.00001) annotation (Placement(transformation(
              extent={{-13,10},{13,-10}},
              rotation=180,
              origin={55,54})));
      equation
        connect(equivalent_Source.positivePlug, vSC_MMC_AVMP.P_Plug) annotation (Line(points={{-88.4,12},{-92,12},{-92,54},{-83.4,54}}, color={0,0,255}));
        connect(resistor.n, vSC_MMC_AVMVDC.P) annotation (Line(points={{16,60},
                {41.6,60},{41.6,59.4}},                                                                color={0,0,255}));
        connect(vSC_MMC_AVMP.P, resistor.p) annotation (Line(points={{-55.6,59.4},{-23.8,59.4},{-23.8,60},{-4,60}}, color={0,0,255}));
        connect(vSC_MMC_AVMP.N, vSC_MMC_AVMVDC.N) annotation (Line(points={{-55.6,
                49.8},{-55.6,49.8},{41.6,49.8}},                                                                  color={0,0,255}));
        connect(groundT5.p, vSC_MMC_AVMVDC.N) annotation (Line(points={{-8,42},
                {-8,49.8},{-5.8,49.8},{41.6,49.8}},                                                              color={0,0,255}));
        connect(vSC_MMC_AVMVDC.P_Plug, equivalent_Source1.positivePlug) annotation (Line(points={{69.4,54},
                {92,54},{92,10},{80.4,10}},                                                                                            color={0,0,255}));
        connect(equivalent_Source.negativePlug, star1.plug_p) annotation (Line(points={{-67.4,12},{-46,12},{-46,0}}, color={0,0,255}));
        connect(equivalent_Source1.negativePlug, star1.plug_p) annotation (Line(points={{59.4,10},{32,10},{-46,10},{-46,0}}, color={0,0,255}));
        connect(groundT2.p, star1.pin_n) annotation (Line(points={{-44,-32},{-44,-32},{-44,-20},{-46,-20}}, color={0,0,255}));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})),experiment(
            StopTime=1.2,
            Tolerance=1e-3,
            __Dymola_fixedstepsize=0.0001,
            __Dymola_Algorithm="Dassl"),
          __Dymola_experimentSetupOutput);
      end TwoNodePowerSystem;

      model IdqRefTest

        HVDC_Emtp.IdqRef_Limiter idqRef_Limiter(Prio=1) annotation (Placement(transformation(extent={{-22,0},{-2,20}})));
        Modelica.Blocks.Sources.Step step(
          height=1,
          offset=0.2,
          startTime=2) annotation (Placement(transformation(extent={{-90,16},{
                  -70,36}})));
        Modelica.Blocks.Sources.Step step1(
          height=0,
          offset=0,
          startTime=2) annotation (Placement(transformation(extent={{-90,-24},{
                  -70,-4}})));
        Modelica.Blocks.Interfaces.RealOutput y annotation (Placement(transformation(extent={{76,14},{96,34}})));
        Modelica.Blocks.Interfaces.RealOutput y1 annotation (Placement(transformation(extent={{76,-16},{96,4}})));
      equation
        connect(step1.y, idqRef_Limiter.Iq_ref_in) annotation (Line(points={{-69,-14},
                {-64,-14},{-54,-14},{-54,5.6},{-24.4,5.6}},                                                                                 color={0,0,127}));
        connect(step.y, idqRef_Limiter.Id_ref_in) annotation (Line(points={{-69,26},
                {-44,26},{-44,13.4},{-24.4,13.4}},                                                                     color={0,0,127}));
        connect(idqRef_Limiter.Id_ref_out, y) annotation (Line(points={{-1,14},{36,14},{36,24},{86,24}}, color={0,0,127}));
        connect(idqRef_Limiter.Iq_ref_out, y1) annotation (Line(points={{-1,6.2},{38,6.2},{38,-6},{86,-6}}, color={0,0,127}));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
      end IdqRefTest;
    end VSC;
  end Example;



  annotation (uses(                           iPSL(version="0.8.1"), Modelica(
          version="3.2.2")),
      Documentation(info="<html>
<table cellspacing=\"2\" cellpadding=\"0\" border=\"0\"<tr>
<td><p>Reference</p></td>
<td><p>EMTP-RV</p></td>
</tr>
<tr>
<td><p>Last update</p></td>
<td><p>30/04/2017</p></td>
</tr>
<tr>
<td><p>Author</p></td>
<td><p>MAA Murad and Luigi Vanfretti, SmarTS Lab, KTH Royal Institute of Technology</p></td>
</tr>
<tr>
<td><p>Contact</p></td>
<td><p><a href=\"mailto:luigiv@kth.se\">luigiv@kth.se</a></p></td>
</tr>
</table>
</html>"));
end VSCHVDC;
