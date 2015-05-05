function phiThetaPsiPto = pqr2phiThetaPsiPto(this,pqr,ptp)
  phiThetaPsiPto = [(pqr(1) + tan(ptp(2)) * (pqr(2) * sin(ptp(1)) + pqr(3)*cos(ptp(1))));
                    (pqr(2) * cos(ptp(1)) +  pqr(3) * sin(ptp(1)))                      ;
                   ((pqr(2) * sin(ptp(1)) +  pqr(3) * cos(ptp(1)))/cos(ptp(2)))        ];
end
