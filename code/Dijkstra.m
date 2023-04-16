% Src: https://gjacopo.github.io/imtools/graph/dijk.html

function [dist, varargout] = dijk(W, start_verts, end_verts)

    [dist, P] = dijk_(W, start_verts, end_verts);
    if nargout==2
        varargout{1} = pred2path_(P, start_verts, end_verts);
    end

end % end of dijk

%--------------------------------------------------------------------------
function [D, P] = dijk_(A,s,t)
    % Copyright (c) 1994-2003 by Michael G. Kay
    % Matlog Version 7 02-Sep-2003
    
    [n,cA] = size(A);
    
    if nargin < 2 || isempty(s), s = (1:n)'; else s = s(:); end
    if nargin < 3 || isempty(t), t = (1:n)'; else t = t(:); end
    
    if ~any(any(tril(A) ~= 0))			% A is upper triangular
       isAcyclic = 1;
    elseif ~any(any(triu(A) ~= 0))	% A is lower triangular
       isAcyclic = 2;
    else										% Graph may not be acyclic
       isAcyclic = 0;
    end
    
    if n ~= cA
       error('A must be a square matrix');
    elseif ~isAcyclic && any(any(A < 0))
       error('A must be non-negative');
    elseif any(s < 1 | s > n)
       error(['''s'' must be an integer between 1 and ',num2str(n)]);
    elseif any(t < 1 | t > n)
       error(['''t'' must be an integer between 1 and ',num2str(n)]);
    end
    
    A = A';		% Use transpose to speed-up FIND for sparse A
    
    D = zeros(length(s),length(t));
    P = zeros(length(s),n);
    
    for i = 1:length(s)
       j = s(i);
    
       Di = Inf*ones(n,1); Di(j) = 0;
    
       isLab = false(length(t),1);
       if isAcyclic ==  1
          nLab = j - 1;
       elseif isAcyclic == 2
          nLab = n - j;
       else
          nLab = 0;
          UnLab = 1:n;
          isUnLab = true(n,1);
       end
    
       while nLab < n && ~all(isLab)
          if isAcyclic
             Dj = Di(j);
          else	% Node selection
             [Dj,jj] = min(Di(isUnLab));
             j = UnLab(jj);
             UnLab(jj) = [];
             isUnLab(j) = 0;
          end
    
          nLab = nLab + 1;
          if length(t) < n, isLab = isLab | (j == t); end
    
          [jA,~,Aj] = find(A(:,j));
          Aj(isnan(Aj)) = 0;
    
          if isempty(Aj), Dk = Inf; else Dk = Dj + Aj; end
    
          P(i,jA(Dk < Di(jA))) = j;
          Di(jA) = min(Di(jA),Dk);
    
          if isAcyclic == 1			% Increment node index for upper triangular A
             j = j + 1;
          elseif isAcyclic == 2	% Decrement node index for lower triangular A
             j = j - 1;
          end
    
          %disp( num2str( nLab ));
       end
       D(i,:) = Di(t)';
    end

end

%--------------------------------------------------------------------------
function rte = pred2path_(P,s,t)
    % Copyright (c) 1994-2006 by Michael G. Kay
    % Matlog Version 9 13-Jan-2006 (http://www.ie.ncsu.edu/kay/matlog)
    
    [~,n] = size(P);
    
    if nargin < 2 || isempty(s), s = (1:n)'; else s = s(:); end
    if nargin < 3 || isempty(t), t = (1:n)'; else t = t(:); end
    
    if any(P < 0 | P > n)
       error(['Elements of P must be integers between 1 and ',num2str(n)]);
    elseif any(s < 1 | s > n)
       error(['"s" must be an integer between 1 and ',num2str(n)]);
    elseif any(t < 1 | t > n)
       error(['"t" must be an integer between 1 and ',num2str(n)]);
    end
    
    rte = cell(length(s),length(t));
    
    [~,idxs] = find(P==0);
    
    for i = 1:length(s)
    %    if rP == 1
    %       si = 1;
    %    else
    %       si = s(i);
    %       if si < 1 | si > rP
    %          error('Invalid P matrix.')
    %       end
    %    end
       si = find(idxs == s(i));
       for j = 1:length(t)
          tj = t(j);
          if tj == s(i)
             r = tj;
          elseif P(si,tj) == 0
             r = [];
          else
             r = tj;
             while tj ~= 0
                if tj < 1 || tj > n
                   error('Invalid element of P matrix found.')
                end
                r = [P(si,tj); r];                                          %#ok
                tj = P(si,tj);
             end
             r(1) = [];
          end
          rte{i,j} = r;
       end
    end
    
    if length(s) == 1 && length(t) == 1
       rte = rte{:};
    end
    
    %rte = t;
    while 0%t ~= s
       if t < 1 || t > n || round(t) ~= t
          error('Invalid "pred" element found prior to reaching "s"');
       end
       rte = [P(t) rte];                                                   %#ok
       t = P(t);
    end
end
