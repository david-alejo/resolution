function [m_ga, m_mc] = get_times_pf(data_ga, data_mc)
% Remember:
% data_ga{n_tests}(n_iterations)
% data_mc{n_tests}(n_iterations+1)
% Time against iterations [parametrized with n_uavs]
color=['b' 'r' 'g' 'k' 'y' 'm' 'b--' 'r--'  'g--' 'k--' 'y--' ];

n_tests = length(data_ga);
n_it = length(data_ga{1});

aux2getMean_ga = zeros(n_tests, n_it);
aux2getMean_mc = zeros(n_tests, n_it);
% From cell to matrix
for m=1:n_tests
  for it=1:n_it
  aux2getMean_ga(m,it)=data_ga{m}(it);
  aux2getMean_mc(m,it)=data_mc{m}(it);
  end
  aux2getMean_mc(m,n_it+1)=data_mc{m}(n_it+1);
end
% Calculate mean
m_ga = mean(aux2getMean_ga);
m_mc = mean(aux2getMean_mc);
