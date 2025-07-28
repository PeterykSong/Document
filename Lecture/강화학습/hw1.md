## Answer 1.

수식 3번에서 시작해보자.
$$
J(\theta) = V^{\pi}(s_0) = \displaystyle\sum_{a}\pi_\theta (a |s_0) Q^\pi(S_0,a) 
$$

여기서 $\nabla_\theta J(\theta)$ 를 표현하면 다음과 같이 쓸 수 있다. 

$$ 
\begin{align*}
\nabla_\theta J(\theta) &= \nabla_\theta \left( \displaystyle\sum_{a}\pi_\theta (a |s_0) Q^\pi(s_0,a) \right)  \\
&=  \displaystyle\sum_{a} \nabla_\theta \left( \pi_\theta (a |s_0) Q^\pi(s_0,a) \right) \\
\end{align*} 
$$ 

이때, $\nabla_\theta \left( \pi_\theta (a |s_0) Q^\pi \right)$ 에 곱의 미분법을 적용하면, 

$$
\nabla_\theta \left( \pi_\theta (a |s_0) Q^\pi (s_0,a)  \right) = \nabla_\theta \pi_\theta (a |s_0) \cdot Q^\pi (s_0,a)  +  \pi_\theta (a |s_0) \cdot \nabla_\theta  Q^\pi (s_0,a) 
$$

으로 정리되는데, 우변의 첫번째 항에 로그 미분 트릭을 적용하면, 첫번째 항을 아래와 같이 바꿔 쓸 수 있다. 

$$ 
 \nabla_\theta \pi_\theta (a |s_0) \cdot Q^\pi = \pi_\theta(a|s_0) \nabla_\theta \log \pi_\theta(a|s_0) \cdot  Q^\pi(s_0,a) \\[2pt]
 \\
\left( \because \nabla_\theta \log \pi_\theta (a|s_0) = \frac{\nabla_\theta \pi_\theta(a|s_0)}{\pi_\theta(a|s_0)} \right)
$$

곱의 미분과, 로그 미분트릭을 모두 적용한  $\nabla_\theta J(\theta)$ 는 다음과 같다. 

$$
\begin{align*}
\nabla_\theta J(\theta) &= \displaystyle\sum_{a} \left[ \pi_\theta(a|s_0) \nabla_\theta \log \pi_\theta(a|s_0) \cdot  Q^\pi(s_0,a) +  \pi_\theta (a |s_0) \cdot \nabla_\theta  Q^\pi (s_0,a)   \right]
\end{align*} 
$$

## Answer 2.
문제의 식 4번과 수식이 일치하지 않는 이유는, Policy Gradient Therom 에서 앞의 1번 답 식의 두번째 항 중  $\nabla_\theta  Q^\pi (s_0,a)$을 0으로 근사하기 때문이다. 

이 근사(Approximation)이 타당하려면, 

 - $\nabla_\theta  Q^\pi (s,a)$ 이 작거나 무시가 가능할때, 
 - $Q^\pi (s,a)$ 이 변화가 없을때
 - Baseline과 같은 기법을 적용한 경우, 
 - On-Sampling 기법을 사용할 경우

