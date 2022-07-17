% func. to cal. resulting intervals of a set of polynomial inequalities
% Ref: e.g., solve eq.(20) or eq.(24)
%
% @ Benji Z. Zhang

function Real_invt = RealRootsIntersectSet(end_time, CoUniPol, inequal)
    min = 0;
    max = end_time; % the end of time
    
    % solve for roots
    real_root_pts = [];
    numIneq = length(CoUniPol); % num of inequalities 
    for i = 1:numIneq
        root_pts_temp = roots(CoUniPol{i}); % column vector
        % leave real soln, eliminate complex soln of EACH polynomials
        real_root_pts_temp = root_pts_temp(imag(root_pts_temp)==0);
        if ~isempty(real_root_pts_temp) % have real solns
            real_root_pts = unique([real_root_pts; real_root_pts_temp]);  % (T)
        else % no real solns
            midpts = (min+max)*0.5;
            polyval_temp = round(polyval(CoUniPol{i}, midpts), 10);
            if strcmp(inequal, '<=')
                if polyval_temp <= 0
                else
                    Real_invt = [];return;
                end
             elseif strcmp(inequal, '>=')
                if polyval_temp >= 0
                else
                    Real_invt = [];return; 
                end
            % -- testing for open interval
            elseif strcmp(inequal, '<')
                if polyval_temp < 0
                else
                    Real_invt = [];return;
                end
            elseif strcmp(inequal, '>')
                if polyval_temp > 0
                else
                    Real_invt = [];return;
                end
            else
                error('Invalid Sign or solution is zero');
            end
        end
    end

    if isempty(real_root_pts) % all inequality solns are [-inf inf]
        Real_invt = [min max];
    else
        if min>= max
            error('Invalid limits of flexible variable');
        end
        Rel_root_pts = real_root_pts((real_root_pts > min) & (real_root_pts < max));

        % judge the positive/negative intervals 
        % if ~isempty(Rel_root_pts) % optimize code for empty result case
        Rel_root_pts = sort(Rel_root_pts);

        % S2 add 2 boundary pts
        % check_pts = [min; Rel_root_pts; max];

        % S3 pick up the MidPts
        % Due to the HIGH degree of 5th polynomial, it will give incorrect
        % results when there is a tiny difference of values in MidPts. So
        % we use "round" to offset the error.
        low = [min; Rel_root_pts];
        up  = [Rel_root_pts; max];
        MidPts = round((low+up)*0.5,4); % vector

        % S4 calculate the polynomial valus according to the MidPts
        PolyVal = zeros(length(MidPts), numIneq);
        for i = 1:numIneq
            polyval_temp = round(polyval(CoUniPol{i}, MidPts), 10); % same as the form of MidPts
            % polyval_temp2 = polyval(CoUniPol{i}, check_pts)
           %     if strcmp(inequal, '<')
            if strcmp(inequal, '<=')
           %         polyval_temp_ind = (polyval_temp<0); % 1 refers to negative ploy-value
                polyval_temp_ind = (polyval_temp<=0);
           %     elseif strcmp(inequal, '>')
            elseif strcmp(inequal, '>=')
           %         polyval_temp_ind = (polyval_temp>0); % 1 refers to positive ploy-value
                polyval_temp_ind = (polyval_temp>=0);
                % ---- testing for open interval
            elseif strcmp(inequal, '<')
                polyval_temp_ind = (polyval_temp<0);
            elseif strcmp(inequal, '>')
                polyval_temp_ind = (polyval_temp>0);                            
                % -----
            else
                error('Invaid inequality');
            end
            PolyVal(:, i) = polyval_temp_ind;
        end

        % S5 collect the neagtive(-) intervals (i.e. IFW intervals) and store in
        % the form of n*2 matrix
        logic = all(PolyVal, 2); % INTERSECTION set of all inequalities
        k = find(logic > 0);
        if isempty(k)
            out_temp = [];
        else
            Left = low(k); % column vector
            Right = up(k); % column vector
            [L_or, R_or] = Or_interval( Left, Right); % avoid adjacent intervals
            out_temp = [L_or', R_or']; % n*2 matrix
        end
        Real_invt = out_temp; % n*2 matrix or null
    end
end